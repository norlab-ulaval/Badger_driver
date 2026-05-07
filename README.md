# STM 32 steering interface

This repo's first objective is to obtain feedback from the vehicule's potentiometer attached to it's atriculation. To accomplish that the stm32 based Nucleo-F303k8 MCU was used you can find the MCU code, config and related documents on in the Badger Steering Folder.

# Ros2 serial interface

This is a simple ros2 package that contains the serial link to the STM32 steering patch this is meant to be a teporary package

# Badger_msgs (TBD)

This is a package that defines the Badger_status topic. This package is meant to expand into a full low-level status update topic collection.

# CAN bus driver (TBD)
The main goal of this project is to develop a CANBus driver capable of commanding the motors on the platform. Communication on a Can bus between the High level computer and a roboteq MDC [Roboteq MDC2460s](https://www.roboteq.com/docman-list/legacy-1/legacy-user-manuals/272-roboteq-controllers-user-manual-v21/file) motor driver and the custom made ESP 32 Motor driver interface.



# ESP 32 Motor driver canbus interface (TBD)

With this interface we aim to attain better control over the [Kelly HPM ](https://kellycontroller.com/shop/hpm/) brushed DC motor driver


