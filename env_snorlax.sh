# Terminal setup on snorlax (flatbot). In every new terminal:  source ~/RAMMP-table/env_snorlax.sh
source ~/ros2_ws/install/setup.zsh
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp TABLE_ARM_BACKEND=kinova \
       TABLE_VZ_STOP=-0.01 TABLE_REACH="0.70,1.10"
alias py=~/table-venv/bin/python      # py place_bowl.py look --no-window
cd ~/RAMMP-table
