# Ubuntu 18.04 with nvidia-docker2 beta opengl support
FROM osrf/subt-virtual-testbed:latest

RUN sed -i 's/<static_update_frequency>1<\/static_update_frequency>/<static_update_frequency>50<\/static_update_frequency>/' /home/developer/subt_ws/src/subt/subt_ign/launch/cave_circuit.ign
RUN sed -i 's/<static_update_frequency>1<\/static_update_frequency>/<static_update_frequency>50<\/static_update_frequency>/' /home/developer/subt_ws/src/subt/submitted_models/costar_husky_sensor_config_1/launch/spawner.rb
RUN /bin/bash -c 'source /opt/ros/melodic/setup.bash && catkin_make -DCMAKE_BUILD_TYPE=Release install'
ENTRYPOINT ["./run_sim.bash"]

# Customize your image here.
# E.g.:
# ENV PATH="/opt/sublime_text:$PATH"
