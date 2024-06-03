FROM aerostack2/nightly-humble:latest

RUN git clone https://github.com/aerostack2/project_gazebo.git ../project_gazebo
WORKDIR /root/project_gazebo

CMD ["bash"]
