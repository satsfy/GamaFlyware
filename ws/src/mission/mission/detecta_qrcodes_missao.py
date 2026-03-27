import sys
import select
import math

import cv2
from ultralytics import YOLO
from cv_bridge import CvBridge
import rclpy
import rclpy.clock

from .fase3_script import NoFase3


SHOULD_SAVE_IMGS = False
MILLISECONDS_IN_NS = 1e6


class DetectaQRCodes(NoFase3):
    """
    Nó do ROS principal que controla nosso drone
    """

    def __init__(self, name: str = "detecta_qrcodes"):
        super().__init__(name)

        self.funcao_estado_atual = self.inicia

        self.origem = [0.22532819211483002, 0.1024281308054924, 0.7090049386024475]

        self.last_image = None
        self.capture_image_flag = False
        self.model = YOLO("src/mission/mission/edra_colab_model.pt", verbose=False)
        self.qr_detector = cv2.QRCodeDetector()

        self.standard_height = 25.0
        self.approach_height = None
        self.approach_height_delta = 0.006
        self.min_base_drone_distance = 1.830000000000019
        self.max_base_drone_distance = 6
        self.approach_height_reduce_last = 0
        self.approach_height_reduce_period = 10 * MILLISECONDS_IN_NS

        self.last_qrcodes = None
        self.alternativas_codigo_qrcodes = ["A", "B", "C", "D", "E"]
        # self.set_frequent_log_ignores(50)
        # self.set_log_level(5)

    def tem_alguma_base_nao_visitada(self) -> bool:
        next_base = self.coordenadas.find_next_base()
        if next_base is None:
            self.logger.info("Todas as bases já estão visitadas")
            return False
        self.logger.info(
            f"Próxima base: {next_base} {self.coordenadas.new_bases[next_base]}"
        )
        return True

    def navegue_para_origem(self):
        x, y, _ = self.origem
        self.publish_position_setpoint(x, y, self.standard_height)

    def esta_em_cima_da_origem(self) -> bool:
        x, y, _ = self.origem
        if self.trajectory.is_in_position_2d(x, y, 0.5):
            return True
        return False

    def esta_segurando_uma_caixa(self) -> bool:
        raise NotImplementedError()
        return True

    def aproxima_da_base(self):
        raise NotImplementedError()

    def navage_para_proxima_base(self):
        x, y, _ = self.coordenadas.get_posicao_base_atual()
        dx, dy, _, yaw = self.trajectory.get_drone_x_y_z_heading(should_fail=False)
        self.logger.info(f"Posicao base   [{x}, {y}]\n")
        self.logger.info(f"Posicao drone ---------------- [{dx}, {dy}]\n")
        # self.publish_position_setpoint_with_interpolation(
        #     x, y, self.standard_height, num_steps=50
        # )
        # self.publish_position_setpoint(x, y, self.standard_height)
        self.publish_position_setpoint2(x, y, self.standard_height)

    def esta_em_cima_da_base_odometria(self) -> bool:
        x, y, _ = self.coordenadas.get_posicao_base_atual()
        # self.custom_sleep(3)
        if self.trajectory.is_in_position_2d(x, y, 1.0):
            for i in range(3):
                self.custom_sleep(1)
                if not self.trajectory.is_in_position_2d(x, y, 1.0):
                    return False
            return True
        return False

    def esta_em_cima_de_base_pela_camera(self) -> bool:
        frame = self.last_image
        height, width, _ = frame.shape
        self.logger.debug(f"image height: {height} width: {width}")

        results = self.model(frame, conf=0.9, verbose=False)
        annotated_frame = results[0].plot()

        bounding_boxes = [caixa.xyxy[0] for caixa in results[0].boxes]

        # calcula centro de todas as bases de deteção no frame
        bases = [
            (
                int((caixa.xyxy[0][0] + caixa.xyxy[0][2]) / 2),
                int((caixa.xyxy[0][1] + caixa.xyxy[0][3]) / 2),
            )
            for caixa in results[0].boxes
        ]

        # # draw center of boxes in boxes
        # for base in bases:
        #     cv2.circle(annotated_frame, base, 5, (0, 0, 255), -1)

        # draw center of image
        cv2.circle(annotated_frame, (width // 2, height // 2), 5, (0, 255, 0), -1)

        sp = self.trajectory.get_drone_x_y_z_heading()

        if SHOULD_SAVE_IMGS:
            cv2.imwrite(
                f"src/mission/mission/saved/{round(sp[0],2)}_{round(sp[1],2)}_{round(sp[2],2)}.png",
                annotated_frame,
            )

        resized_frame = cv2.resize(annotated_frame, (width, height))
        cv2.imshow(f"BUSCANDO BASE...", resized_frame)
        cv2.waitKey(1)

        self.logger.info(f"{len(bases)} bases detectadas")
        if len(bases) == 0:
            return False

        # verifica que o centro da camera está dentro do bounding box da base
        # caso não esteja, é possível que estamos observando outra base na camera
        center_image = (width // 2, height // 2)
        for base in bounding_boxes:
            x0, y0, x1, y1 = base
            x0, x1 = sorted([x0, x1])
            y0, y1 = sorted([y0, y1])
            if not (x0 < center_image[0] < x1 and y0 < center_image[1] < y1):
                return False

        return True

    def centralizar_posicao_sobre_a_base(self):
        pass

    def afaste_drone_para_encontrar_base(self):
        self.logger.debug("afaste_drone_para_encontrar_base")
        if self.approach_height is None:
            self.logger.debug("approach_height is None")
            self.approach_height = self.standard_height

        if (
            self.approach_height_reduce_last + self.approach_height_reduce_period
            < self.get_timestamp_nanoseconds()
        ):
            self.approach_height_reduce_last = self.get_timestamp_nanoseconds()
            self.approach_height += self.approach_height_delta
            self.logger.info(
                f"Afastando-se para encontrar base. z={self.approach_height}"
            )

        # mas se o chao estiver perto demais do drone, vamos parar de aproximar
        _, _, base_z = self.coordenadas.get_posicao_base_atual()
        x, y, drone_z, _ = self.trajectory.get_drone_x_y_z_heading()
        self.logger.debug(
            "math.fabs(base_z - drone_z) < self.max_base_drone_distance",
            math.fabs(base_z - drone_z) > self.max_base_drone_distance,
        )
        if math.fabs(base_z - drone_z) > self.max_base_drone_distance:
            self.logger.info("Base está muito longe do drone, parando de afastar")
            self.approach_height = base_z + self.max_base_drone_distance

        self.publish_position_setpoint(x, y, self.approach_height)

    def marque_base_como_visitada(self):
        self.coordenadas.marca_base_como_visitada()

    def algum_qrcode_detectado(self) -> bool:
        # return False
        frame = self.last_image
        height, width, _ = frame.shape
        _, decoded_info, points, _ = self.qr_detector.detectAndDecodeMulti(frame)

        if points is not None:
            for bbox, data in zip(points, decoded_info):
                bbox = bbox.astype(int)
                n = len(bbox)
                for i in range(n):
                    start_point = tuple(bbox[i])
                    end_point = tuple(bbox[(i + 1) % n])
                    cv2.line(frame, start_point, end_point, (255, 0, 0), 2)

                # Label the QR code bounding box with the text in 'data'
                if data:
                    # Calculate the position for the text label
                    # Add background color to the text label
                    FONT_SIZE = 2.2
                    text_size, _ = cv2.getTextSize(
                        data, cv2.FONT_HERSHEY_SIMPLEX, FONT_SIZE, 2
                    )
                    text_position = tuple(bbox[0])
                    cv2.rectangle(
                        frame,
                        (text_position[0], text_position[1] - text_size[1]),
                        (text_position[0] + text_size[0], text_position[1]),
                        (255, 0, 0),
                        -1,
                    )
                    cv2.putText(
                        frame,
                        data,
                        text_position,
                        cv2.FONT_HERSHEY_SIMPLEX,
                        FONT_SIZE,
                        (255, 255, 255),
                        2,
                        cv2.LINE_AA,
                    )

                if not data or not data in self.alternativas_codigo_qrcodes:
                    # vamos forçar o sistema a continuar tentando ler qrcodes,
                    # porque não detectou corretamente este qrcode
                    return False

        # Display the resulting frame

        qrcodes = len(points) if points is not None else 0

        resized_frame = cv2.resize(frame, (width, height))
        cv2.imshow(f"QR Code Detector", resized_frame)
        cv2.waitKey(1)

        if points is not None and qrcodes > 0:
            self.last_qrcodes = zip(points, decoded_info)
            return True
        return False

    def sobe_da_base_depois_de_ler_qrcodes(self):
        self.approach_height = None
        x, y, _ = self.coordenadas.get_posicao_base_atual()
        self.publish_position_setpoint(x, y, self.standard_height, force=True)

    def conseguiu_processar_qrcodes_detectados(self):
        if self.last_qrcodes is None:
            return False
        last_qrcode = self.last_qrcodes
        detected = False
        for bbox, data in last_qrcode:
            detected = True
            # self.set_log_level(1)
            self.logger.fatal(f"DETECTED QRCODE !!!!!!!!!!!!!!!: {data}")
        return detected

    def camera_callback(self, msg):
        self.logger.debug(
            f"[CAMERA] camera_callback called with self.capture_image_flag={self.capture_image_flag}"
        )
        self.last_image = CvBridge().imgmsg_to_cv2(msg, "bgr8")

    def aproximar_do_chao_para_ler_qrcode(self):
        if self.approach_height is None:
            self.logger.debug("approach_height is None")
            self.approach_height = self.standard_height

        if (
            self.approach_height_reduce_last + self.approach_height_reduce_period
            < self.get_timestamp_nanoseconds()
        ):
            self.approach_height_reduce_last = self.get_timestamp_nanoseconds()
            self.approach_height -= self.approach_height_delta
            self.logger.debug(
                "Aproximando da base pra ler qrcode.... ALTURA: %f",
                self.approach_height,
            )

        # mas se o chao estiver perto demais do drone, vamos parar de aproximar
        x, y, base_z = self.coordenadas.get_posicao_base_atual()
        _, _, drone_z, _ = self.trajectory.get_drone_x_y_z_heading()
        self.logger.info(
            "math.fabs(base_z - drone_z) < self.min_base_drone_distance: %f",
            math.fabs(base_z - drone_z) < self.min_base_drone_distance,
        )
        if math.fabs(base_z - drone_z) < self.min_base_drone_distance:
            self.logger.debug("Base está muito perto do drone, parando de aproximar")
            self.approach_height = base_z + self.min_base_drone_distance

        self.publish_position_setpoint(x, y, self.approach_height)

    def echo_camera(self):
        try:
            frame = self.last_image
            height, width, _ = frame.shape
            self.logger.debug(f"image height: {height} width: {width}")
            resized_frame = cv2.resize(frame, (int(480 * 16 / 9), 480))
            cv2.imshow(f"BUSCANDO BASE...", resized_frame)
            cv2.waitKey(1)
        except Exception as e:
            self.logger.error(f"SEM IMAGEM DA CAMERA >>>>> Error: {e}")

    # def get_maquina_de_estados(self) -> dict[callable, dict]:
    #     return {
    #         # self.inicia: {"proximo": {"funcao": self.echo_camera}},
    #         # self.echo_camera: {"proximo": {"funcao": self.echo_camera}},
    #         self.inicia: {"proximo": {"funcao": self.engage_offboard_mode}},
    #         self.echo_camera: {
    #             "proximo": {"funcao": self.tem_alguma_base_nao_visitada}
    #         },
    #         self.engage_offboard_mode: {"proximo": {"funcao": self.armar}},
    #         self.armar: {"proximo": {"funcao": self.sobe_voo}},
    #         self.sobe_voo: {
    #             "proximo": {"funcao": self.esta_no_ar_estavel},
    #         },
    #         self.esta_no_ar_estavel: {
    #             "sim": {"funcao": self.tem_alguma_base_nao_visitada},
    #             "nao": {"funcao": self.engage_offboard_mode},
    #         },
    #         self.tem_alguma_base_nao_visitada: {
    #             "sim": {"funcao": self.navage_para_proxima_base},
    #             "nao": {"funcao": self.navegue_para_origem},
    #         },
    #         self.navage_para_proxima_base: {
    #             "proximo": {"funcao": self.esta_em_cima_da_base_odometria},
    #         },
    #         # self.navage_para_proxima_base: {
    #         #     "proximo": {"funcao": self.echo_camera},
    #         # },
    #         # self.esta_em_cima_da_base_odometria: {
    #         #     "sim": {"funcao": self.esta_em_cima_de_base_pela_camera},
    #         #     "nao": {"funcao": self.navage_para_proxima_base},
    #         # },
    #         self.esta_em_cima_da_base_odometria: {
    #             "sim": {"funcao": self.marque_base_como_visitada},
    #             "nao": {"funcao": self.navage_para_proxima_base},
    #         },
    #         self.esta_em_cima_de_base_pela_camera: {
    #             "sim": {"funcao": self.centralizar_posicao_sobre_a_base},
    #             "nao": {"funcao": self.afaste_drone_para_encontrar_base},
    #         },
    #         self.centralizar_posicao_sobre_a_base: {
    #             "proximo": {"funcao": self.marque_base_como_visitada}
    #         },
    #         self.afaste_drone_para_encontrar_base: {
    #             "proximo": {"funcao": self.esta_em_cima_de_base_pela_camera},
    #         },
    #         # self.marque_base_como_visitada: {
    #         #     "proximo": {"funcao": self.algum_qrcode_detectado},
    #         # },
    #         self.marque_base_como_visitada: {
    #             "proximo": {"funcao": self.tem_alguma_base_nao_visitada},
    #         },
    #         self.aproximar_do_chao_para_ler_qrcode: {
    #             "proximo": {"funcao": self.algum_qrcode_detectado},
    #         },
    #         self.algum_qrcode_detectado: {
    #             "sim": {"funcao": self.sobe_da_base_depois_de_ler_qrcodes},
    #             "nao": {"funcao": self.aproximar_do_chao_para_ler_qrcode},
    #         },
    #         self.sobe_da_base_depois_de_ler_qrcodes: {
    #             "proximo": {"funcao": self.conseguiu_processar_qrcodes_detectados},
    #         },
    #         self.conseguiu_processar_qrcodes_detectados: {
    #             "sim": {"funcao": self.tem_alguma_base_nao_visitada},
    #             "nao": {"funcao": self.aproximar_do_chao_para_ler_qrcode},
    #         },
    #         self.navegue_para_origem: {
    #             "proximo": {"funcao": self.esta_em_cima_da_origem},
    #         },
    #         self.esta_em_cima_da_origem: {
    #             "sim": {"funcao": self.pousa},
    #             "nao": {"funcao": self.navegue_para_origem},
    #         },
    #         self.pousa: {"proximo": {"funcao": self.disarma}},
    #         self.disarma: {"proximo": {"funcao": self.inicia}},
    #     }

    def get_maquina_de_estados(self) -> dict[callable, dict]:
        return {
            self.inicia: {"proximo": {"funcao": self.engage_offboard_mode}},
            self.engage_offboard_mode: {"proximo": {"funcao": self.armar}},
            self.armar: {"proximo": {"funcao": self.sobe_voo}},
            self.sobe_voo: {
                "proximo": {"funcao": self.esta_no_ar_estavel},
            },
            self.esta_no_ar_estavel: {
                "sim": {"funcao": self.tem_alguma_base_nao_visitada},
                "nao": {"funcao": self.engage_offboard_mode},
            },
            self.tem_alguma_base_nao_visitada: {
                "sim": {"funcao": self.navage_para_proxima_base},
                "nao": {"funcao": self.navage_para_proxima_base},
            },
            self.navage_para_proxima_base: {
                "proximo": {"funcao": self.esta_em_cima_da_base_odometria},
            },
            self.esta_em_cima_da_base_odometria: {
                "sim": {"funcao": self.marque_base_como_visitada},
                "nao": {"funcao": self.navage_para_proxima_base},
            },
            self.marque_base_como_visitada: {
                "proximo": {"funcao": self.tem_alguma_base_nao_visitada},
            },
        }


def is_data():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])


def main(args=None) -> None:
    rclpy.init(args=args)
    offboard_control = DetectaQRCodes()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(e)
