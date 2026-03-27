# /root/.bashrc sources this file by default

if [[ "${BASH_SOURCE[0]}" != "$0" ]]; then
    _run_px4_sitl_completion() {
        local cur opts
        COMPREPLY=()
        cur="${COMP_WORDS[COMP_CWORD]}"
        # List all .sdf files in /sitl/gz/worlds, remove directory and .sdf extension
        opts=$(ls /sitl/gz/worlds/*.sdf 2>/dev/null | xargs -n1 basename | sed 's/\.sdf$//')
        COMPREPLY=( $(compgen -W "${opts}" -- "${cur}") )
    }
    # Register completion for the script by its basename.
    complete -F _run_px4_sitl_completion run_px4_sitl.sh
fi