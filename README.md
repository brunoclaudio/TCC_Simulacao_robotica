# TCC_Simulacao_robotica

Referencia Geral: https://github.com/ros-industrial/abb_experimental.git

##Resumo: 
Este repositorio GIT foi criado para organizr e gerenciar os arquivos para executar a simulacao do Abb_irb1200_5/90 no ambiente Gazebo. Os arquivos modificados serao comentados neste Read.me. O trabalho tem como objetivo simular ete robo para executar tres tipos de trajetoria: 
1) Ponto-a-Ponto 
2) Trajetoria Parametrizada e 
3) Trajetoria em linha reta. 

Foi utilizado o framework Moveit! como ferramenta principal para a configracao e planejamento de Trajetorias, em especifico, foi utilizado o move_group_python.API para enviar comandos para o robo. Para o controle de trajetoria, foi escolhido o package Ros_control e definido como parametro de controle o Joint Trajectory Control.  

