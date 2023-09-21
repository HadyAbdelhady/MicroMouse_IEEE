This code represents our entry into the IEEE VICTORIS tournament, where we advanced to the finals of their prestigious sponsorship program for the most innovative solution to a maze-solving challenge. The primary codebase is written in the C programming language and runs on the ATmega32 microcontroller chip by Microchip.

Our team, consisting of myself and my teammate Amir, is proud to have developed all the necessary drivers from scratch to ensure full control and customization of the hardware components. If you wish to contact Amir, his contact details can be found at
Git: https://github.com/AmirBasiony.
LinkedIn: https://www.linkedin.com/in/amir-elbasiony-57809b204/.
The drivers we meticulously implemented include:

- MCAL (Microcontroller Abstraction Layer):
  - GPIO (General-Purpose Input/Output)
  - EEPROM (Electrically Erasable Programmable Read-Only Memory)
  - TIMER0/TIMER1
  - Interrupt handling
  - UART (Universal Asynchronous Receiver-Transmitter)

- HAL (Hardware Abstraction Layer):
  - Ultrasonic sensor integration
  - MPU-5060 sensor (although it was ultimately unused)

For our maze-solving algorithm, we crafted a Floodfill algorithm from the ground up. However, we chose to implement it using matrix structures instead of a queue. To enhance our robot's navigation capabilities within the maze, we introduced a tracking system.

Our hardware setup included an HC-SR04 ultrasonic sensor to detect walls within the robot's environment. We opted for DC motors equipped with magnetic encoders built-in. Although we initially experimented with optical encoders, their accuracy did not meet our requirements.

To ensure precise control over the motors, we implemented a PID (Proportional-Integral-Derivative) control system, which effectively managed the robot's movements throughout the maze.
While our project achieved success in terms of software development and algorithmic implementation, it's essential to acknowledge its downsides. One significant limitation was the mechanical design aspect, which posed challenges due to time constraints. Unfortunately, this constraint resulted in suboptimal behavior exhibited by the robot, particularly in the middle stages of the maze-solving task.

It's worth noting that the algorithm we devised and implemented performed admirably, demonstrating the effectiveness of our software solutions. If you're interested, you can explore a simulation of our project in the "mms" folder, which offers valuable insights into our approach and its outcomes.

In retrospect, this project serves as a valuable learning experience, highlighting the importance of comprehensive design considerations, including mechanical aspects, in robotics and automation projects. Moving forward, we are committed to refining our skills and knowledge to address such challenges effectively and produce even more robust and successful solutions.

This project exemplifies our dedication to developing a comprehensive and innovative solution for maze-solving, showcasing our proficiency in both hardware and software design. We are grateful for the opportunity to have participated in the IEEE VICTORIS tournament and look forward to future challenges in the field of robotics and automation.