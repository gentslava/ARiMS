#!/usr/bin/env python3

from b0lib import b0RemoteApi
import time
import math

"""
Программа для проверки работоспособности связки: 
    {
        CoppeliaSim,
        RemoteApiServer,
        библиотека RemoteApi,
        пользовательская программа
    }

- сцена: scenes\arims_testScene_01.ttt
- название канала: RemoteApi_lab1
"""

# Название канала передачи (Channel name) у объекта сцены  "remote Api server.ttm"
ChannelName = "RemoteApi_lab1"

# Установить связь с симулятором на указанном канале
with b0RemoteApi.RemoteApiClient('b0RemoteApi_pythonClient', ChannelName) as client:

    #####################################################################################
    #####################################################################################
    def myOwnActions():
        motor_L=client.getObjectHandle("revJoint_wheelL")
        motor_R=client.getObjectHandle("revJoint_wheelR")
        visSensor_L=client.getObjectHandle("Vision_sensorL") 
        visSensor_R=client.getObjectHandle("Vision_sensorR")

        isErrL, visArr_L = client.readVisionSensor(visSensor_L)
        isErrR, visArr_R = client.readVisionSensor(visSensor_R)

        if  visArr_L == nil  and  visArr_R == nil:
             
            err = visArr_L[11] - visArr_R[11]
            print("  visArr L=", visArr_L[11], "; R =", visArr_R[11], "; err =", err)
        
            client.setJointTargetVelocity(motor_L, math.pi*visArr_L[11])
            client.setJointTargetVelocity(motor_R, math.pi*visArr_R[11])
            if visArr_R[11] <= 0.2:
                client.setJointTargetVelocity(motor_L, 1.0)
                client.setJointTargetVelocity(motor_R, 0.0)

            if visArr_L[11] <= 0.2:
                client.setJointTargetVelocity(motor_L, 0.0)
                client.setJointTargetVelocity(motor_R, 1.0)
        pass
    #####################################################################################
    #####################################################################################

    def stepSimulation():
        """
        Функция обработки вычислений между кадрами симуляции
        :return:
        """
        if client.runInSynchronousMode:

            while not client.doNextStep:
                client.simxSpinOnce()  # Передать значения.  отрабатывают все CallBack

            # Приостанавливает симуляцию. Доступно новое состояние
            client.doNextStep = False

            ############################################################
            ############################################################
            ##time.sleep(0.5)  # пауза между кадрами симуляции
            myOwnActions()
            ############################################################
            ############################################################

            client.simxSynchronousTrigger()  # Продолжает симуляцию на dt=50 мс (по умолчанию) . Движение сцены
        else:
            client.simxSpinOnce()  # Передать значения.  отрабатывают все CallBack


    # ------------------------------------------------------------------------------------------------------------------
    # Объявление вспомогательных функций

    def simulationStepStarted(msg):
        """
        Действия при начала кадра симуляции
        """
        simTime = msg[1][b'simulationTime']
        print('Simulation step STARTED. Simulation time: {:<5.3f} {:>5.3f}'.format(simTime, time.time() - client.startTime))



    def simulationStepDone(msg):
        """
        Действия при приостановке кадра симуляции
        """
        simTime = msg[1][b'simulationTime']
        print(
            'Simulation step DONE.    Simulation time: {:<5.3f} {:>5.3f}'.format(simTime, time.time() - client.startTime));
        client.doNextStep = True
        print("simulationStepDone", client.doNextStep)


    # ------------------------------------------------------------------------------------------------------------------
    # Остановить симуляцию
    client.simxStopSimulation(client.simxDefaultPublisher())

    if client != -1:
        # При успешном соединении вывод соощения в симуляторе
        print("+++ Соединение УСТАНОВЛЕНО с удаленным API-server +++")
        client.simxAddStatusbarMessage(
            "<br><br> <font color=\'red\'>+++ Соединение с управляющей Python-программой: УСТАНОВЛЕНО  +++</font>@html",
            client.simxDefaultPublisher())


    # Настройка параметров
    client.doNextStep = True  # флаг разрешения выполнения шага симуляции
    client.runInSynchronousMode = True  # флаг работы в синхронном режиме

    # Получение управляющих конструкций узлов модели и проверка их наличия
    isNoErr_rj, client.rev_j_Handle = client.simxGetObjectHandle('Revolute_joint', client.simxServiceCall())
    assert isNoErr_rj, ("!!! Не найден узел модели с названием - Revolute_joint")

    # Привязка функций обработчиков
    client.simxGetSimulationStepStarted(client.simxDefaultSubscriber(simulationStepStarted))
    client.simxGetSimulationStepDone(client.simxDefaultSubscriber(simulationStepDone))

    # Запустить симуляцию в синхронном режиме
    client.simxSynchronous(True)
    client.simxStartSimulation(client.simxDefaultPublisher())



    ############################################################
    # Выполнять в течении ___ секунд
    client.startTime = time.time()  # время начала работы
    while time.time() < client.startTime + 5:
        ##time.sleep(0.5)  # пауза между кадрами симуляции
        stepSimulation()
    ############################################################

    # Остановка вращения узла
    client.simxSetJointTargetVelocity(client.rev_j_Handle, 0, client.simxDefaultPublisher())

    # Остановить симуляцию
    client.simxStopSimulation(client.simxDefaultPublisher())
