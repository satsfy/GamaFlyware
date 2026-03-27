import sys
import select

import rclpy
import rclpy.clock

from .coordenadas import Coordenadas
from .constants import TAKEOFF_HEIGHT
from .ros_config import RosConfig

class NoFase3(RosConfig):
    def __init__(self, name: str = "mission", override_bases=None):
        super().__init__(name)

        self.funcao_estado_atual = self.inicia

        self.coordenadas = Coordenadas(override_bases)

    def loop_principal(self):
        self.publish_offboard_control_heartbeat_signal()
        self.verificar_erro()

        self.logger.info(
            f">>>>>>>>>>> Chamando função: {self.funcao_estado_atual.__name__}"
        )
        output_funcao = self.funcao_estado_atual()

        self.verificar_erro()

        estado = self.get_maquina_de_estados()[self.funcao_estado_atual]
        funcao_proximo_estado = None

        if "proximo" in estado:
            funcao_proximo_estado = estado["proximo"]["funcao"]
        elif "sim" in estado and "nao" in estado:
            if output_funcao is True:
                self.logger.info("Output da função booleana: True")
                funcao_proximo_estado = estado["sim"]["funcao"]
            elif output_funcao is False:
                self.logger.info("Output da função booleana: False")
                funcao_proximo_estado = estado["nao"]["funcao"]
            else:
                self.estado_de_erro(f"Output da função não é booleano: {output_funcao}")
        else:
            self.estado_de_erro("Estado não possui próximo estado")

        if funcao_proximo_estado is None:
            self.estado_de_erro("Proximo estado indefinido")

        # self.logger.info(f"Define proximo estado como: {funcao_proximo_estado.__name__}")
        self.funcao_estado_atual = funcao_proximo_estado

    def estado_de_erro(self, erro):
        # TODO: Implementar estado de erro
        self.get_logger().error(f"Estado de erro ===============")
        self.get_logger().error(f"erro: {erro}")
        exit(1)

    def verificar_erro(self):
        # TODO: Implementar verificador de erro
        erro = False
        if erro:
            self.estado_de_erro("Erro detectado")

    def inicia(self):
        self.logger.info("Inicia called")

    def armar(self):
        self.arm()

    def disarma(self):
        self.disarm()

    def pousa(self):
        self.land()
        self.custom_sleep(7)

    def sobe_voo(self):
        x, y, z, _ = self.trajectory.get_drone_x_y_z_heading(should_fail=False)
        self.get_logger().info(
            f"Envia comando de decolar para x={x} y={y} z={TAKEOFF_HEIGHT} "
        )
        self.get_logger().info(f"Posição atual                 x={x} y={y} z={z} ")
        self.publish_position_setpoint(x, y, TAKEOFF_HEIGHT, force=True)
        self.custom_sleep(3)

    def esta_no_ar_estavel(self) -> bool:
        segundos_de_estabilidade = 1
        distancia_maxima_de_estabilidade = 4  # metro
        tempo_atual = self.get_timestamp_nanoseconds()
        if self.trajectory.is_stable(
            segundos_de_estabilidade,
            distancia_maxima_de_estabilidade,
            tempo_atual,
        ):
            spx, spy, spz = self.get_last_setpoint_x_y_z()
            try:
                if self.trajectory.is_in_position_3d(spx, spy, spz, 0.1):
                    return True
            except Exception as e:
                self.logger.error(e)
                pass
        return False

    def tem_alguma_caixa_nao_visitada(self) -> bool:
        if self.coordenadas.find_next_base() is None:
            self.get_logger().info("Todas as caixas já estão visitadas")
            return False
        return True

    def navegue_para_base(self):
        x, y, _ = self.coordenadas.get_posicao_base_atual()
        self.publish_position_setpoint(x, y, TAKEOFF_HEIGHT)

    def esta_em_cima_da_base(self) -> bool:
        x, y, _ = self.coordenadas.get_posicao_base_atual()
        if self.trajectory.is_in_position_2d(x, y, 0.5):
            return True
        return False

    def esta_segurando_uma_caixa(self) -> bool:
        raise NotImplementedError()
        return True

    def aproxima_da_base(self):
        raise NotImplementedError()

    def libera_a_caixa(self):
        raise NotImplementedError()

    def navage_para_proxima_caixa(self):
        raise NotImplementedError()

    def esta_em_cima_da_caixa_odometria(self) -> bool:
        raise NotImplementedError()
        return True

    def esta_em_cima_da_caixa_camera(self) -> bool:
        raise NotImplementedError()
        return True

    def marque_caixa_como_visitada(self):
        raise NotImplementedError()

    def aproximar_da_caixa(self):
        raise NotImplementedError()

    def ligue_o_ima(self):
        raise NotImplementedError()

    def levanta_voo(self):
        # literalmente a mesma coisa que self.sobe_voo(), mas é
        # um nó diferente
        self.sobe_voo()

    def esta_carregando_caixa(self) -> bool:
        raise NotImplementedError()
        return True

    def ajustar_aproximacao(self):
        raise NotImplementedError()

    def camera_callback(self, msg):
        pass

    def get_maquina_de_estados(self) -> dict[callable, dict]:
        return {
            self.inicia: {
                "proximo": {
                    "funcao": self.engage_offboard_mode,
                    "COMENTARIO": "ESTADO INICIAL ===========",
                },
            },
            self.disarma: {
                "proximo": {
                    "funcao": self.inicia,
                    "COMENTARIO": "ESTADO FINAL =============",
                },
            },
            self.pousa: {
                "proximo": {"funcao": self.disarma},
            },
            self.engage_offboard_mode: {
                "proximo": {"funcao": self.armar},
            },
            self.armar: {
                "proximo": {"funcao": self.sobe_voo},
            },
            self.sobe_voo: {
                "proximo": {"funcao": self.esta_no_ar_estavel},
            },
            self.esta_no_ar_estavel: {
                "sim": {"funcao": self.tem_alguma_caixa_nao_visitada},
                "nao": {"funcao": self.sobe_voo},
            },
            self.tem_alguma_caixa_nao_visitada: {
                "sim": {"funcao": self.navage_para_proxima_caixa},
                "nao": {"funcao": self.navegue_para_base},
            },
            self.navegue_para_base: {
                "proximo": {"funcao": self.esta_em_cima_da_base},
            },
            self.esta_em_cima_da_base: {
                "sim": {"funcao": self.esta_segurando_uma_caixa},
                "nao": {"funcao": self.navegue_para_base},
            },
            self.esta_segurando_uma_caixa: {
                "sim": {"funcao": self.aproxima_da_base},
                "nao": {"funcao": self.pousa},
            },
            self.aproxima_da_base: {
                "proximo": {"funcao": self.libera_a_caixa},
            },
            self.libera_a_caixa: {
                "proximo": {"funcao": self.engage_offboard_mode},
            },
            self.navage_para_proxima_caixa: {
                "proximo": {"funcao": self.esta_em_cima_da_caixa_odometria},
            },
            self.esta_em_cima_da_caixa_odometria: {
                "sim": {"funcao": self.esta_em_cima_da_caixa_camera},
                "nao": {"funcao": self.navage_para_proxima_caixa},
            },
            self.esta_em_cima_da_caixa_camera: {
                "sim": {"funcao": self.marque_caixa_como_visitada},
                "nao": {"funcao": self.navage_para_proxima_caixa},
            },
            self.marque_caixa_como_visitada: {
                "proximo": {"funcao": self.aproximar_da_caixa},
            },
            self.aproximar_da_caixa: {
                "proximo": {"funcao": self.ligue_o_ima},
            },
            self.ligue_o_ima: {
                "proximo": {"funcao": self.levanta_voo},
            },
            self.levanta_voo: {
                "proximo": {"funcao": self.esta_carregando_caixa},
            },
            self.esta_carregando_caixa: {
                "sim": {"funcao": self.navegue_para_base},
                "nao": {"funcao": self.ajustar_aproximacao},
            },
        }


def is_data():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])


def main(args=None) -> None:
    # print("Waiting for key press (f) to start offboard control node...")
    # # Configura o terminal para ler entrada sem bloqueio
    # old_settings = termios.tcgetattr(sys.stdin)
    # tty.setcbreak(sys.stdin.fileno())

    # try:
    #     while True:
    #         if is_data():
    #             c = sys.stdin.read(1)
    #             if c == "f":
    #                 print("Key f pressed, starting offboard control node...")
    rclpy.init(args=args)
    offboard_control = NoFase3()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()
    # break
    # elif c == "\x1b":  # Se 'Esc' for pressionado, encerra o loop
    #     break
    # finally:
    #     # Restaura as configurações do terminal
    #     termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(e)
