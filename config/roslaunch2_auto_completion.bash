# Copied and modified from _roscomplete_launch() in /opt/ros/kinetic/share/rosbash/rosbash:
function _roscomplete_roslaunch2 {
    # Current token (e. g., when the user already typed "roslaunch2 -he", then arg is "-he"):
    arg="${COMP_WORDS[COMP_CWORD]}"
    # Previous token (e. g., when the user already typed "roslaunch2 --help ", then prev is set
    # to "--hel"):
    prev="${COMP_WORDS[COMP_CWORD-1]}"

    # Reset suggestions (will be updated below). This variable must be set to the possibilities for
    # completion; it is consumed by 'complete' at the end. This is done using the 'compgen'
    # function which generate possible completion matches, also refer to
    # https://www.gnu.org/software/bash/manual/html_node/Programmable-Completion-Builtins.html
    COMPREPLY=()

    # List all supported flags in roslaunch and roslaunch2:
    roslaunch_default_flags="--files --args --nodes --find-node --child --local --screen --server_uri --run_id --wait --port --core --pid --dump-params --skip-log-check --ros-args"
    roslaunch_meta_flags="--help --version  --no-colors --keep --dry-run --list --dir --doc"
    roslaunch_meta_flags_short="-h -? -v -"

    # This implements the completion for the various token that havev already been typed:
    if [[ ${arg} =~ \-.* ]]; then # flags beginning with a dash ('-')
        COMPREPLY=(${COMPREPLY[@]} $(compgen -W "$roslaunch_default_flags $roslaunch_meta_flags $roslaunch_meta_flags_short" -- ${arg}))
    elif [[ ${prev} =~ \-\-doc ]]; then # --doc <FUNC> option
        local avail_api=$(roslaunch_meta --list --no-colors | awk '{print $1;}' | cut -d\  -f1 | sort | xargs)
        COMPREPLY=( $(compgen -W "${avail_api}" -- ${arg}) )
    elif [[ ${prev} =~ \-\-dir ]]; then # --dir <PATH> option
        local IFS=$'\n'
        COMPREPLY=( $( compgen -o plusdirs -- $arg ) )
    else # <package> <meta_launch_file>
        _roscomplete_search_dir "( -type f -regex .*\.lmeta$ )"
        if [[ $COMP_CWORD == 1 ]]; then
           COMPREPLY=($(compgen -o plusdirs -f -X "!*.launch" -- ${arg}) ${COMPREPLY[@]} $(compgen -o plusdirs -f -X "!*.test" -- ${arg}) ${COMPREPLY[@]})
        fi
        # complete roslaunch arguments for a launch file
        if [[ ${#COMP_WORDS[@]} -ge 2 ]]; then
            ROSLAUNCH_COMPLETE=$(which roslaunch-complete)
            if [[ -x ${ROSLAUNCH_COMPLETE} ]]; then
                # Call roslaunch-complete instead of roslaunch to get arg completion
                _roslaunch_args=$(${ROSLAUNCH_COMPLETE} ${COMP_WORDS[@]:1:2} 2> /dev/null)
                # roslaunch-complete should be very silent about
                # errors and return 0 if it produced usable completion.
                if [[ $? == 0 ]]; then
                    COMPREPLY=($(compgen -W "${_roslaunch_args}" -- "${arg}") ${COMPREPLY[@]})
                    # FIXME maybe leave ${COMPREPLY[@]} out if we completed successfully.
                fi
            fi
        fi
    fi
}

# Activate completion support for roslaunch_meta:
complete -F "_roscomplete_roslaunch2" -o filenames "roslaunch2"
