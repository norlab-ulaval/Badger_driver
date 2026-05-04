# STM 32 steering serial interface

This repo's first objective is to obtain feedback from the vehicule's potentiometer attached to it's atriculation. To accomplish that the stm32 based Nucleo-F303k8MCU was used you can find the related code in the serial steering folder

# CAN bus driver
The main goal of this project is to develop a CANBus driver capable of commanding the motors on the platform. Communication on a Can bus between the High level computer and a roboteq MDC [Roboteq MDC2460s](https://www.roboteq.com/docman-list/legacy-1/legacy-user-manuals/272-roboteq-controllers-user-manual-v21/file) motor driver and the custom made ESP 32 Motor driver interface.



# ESP 32 Motor driver canbus interface

With this interface we aim to attain better control over the [Kelly HPM ](https://kellycontroller.com/shop/hpm/) brushed DC motor driver


