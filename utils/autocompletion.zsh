autoload -U +X compinit && compinit
autoload -U +X bashcompinit && bashcompinit

this_dir=`cd $(dirname $0); pwd`
source $this_dir/autocompletion.bash