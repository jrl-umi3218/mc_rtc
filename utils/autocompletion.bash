this_dir=`cd $(dirname $0); pwd`
mc_rtc_dir=`cd $this_dir/..; pwd`

_build_and_install_completion()
{
  _script_commands=$($mc_rtc_dir/utils/build_and_install.sh --inputlist)
  local cur
  COMPREPLY=()
  cur="${COMP_WORDS[COMP_CWORD]}"
  COMPREPLY=( $(compgen -W "${_script_commands}" -- ${cur}) )
}
complete -o nospace -F _build_and_install_completion build_and_install.sh