# D-H approach to forward kinematics: practicing pointers and matrixes in C

This was a college assignment for the discipline Algorithms and Data Structures. I was asked to "propose a project in C and define its requirements".

My final paper at IFRS (where I earned my technical certificate in Industrial Automation) consisted of an intelligent robot for object classification. This project is better described and documented in the Intelligent-robot-for-object-classification-and-separation repository. Aside from the machine learning algorithm, I was also the one responsible for the robot kinematics and its trajectory planning. 

Therefore, when the assignment was proposed I was already pretty familiar with Denavit-Hartenberg approach to forward kinematics. Since I'd just learned how to use pointers and manipulate matrixes in C, I thought this topic would be great for me to train my skills. Because of that, I proposed a SCARA robot's control program in which the user would be able to configure some initial parameters and to move the robot's joints during the execution. The goal was to read the user's input and, using D-H approach to forward kinematics, determine the position of the end-effectuator. The base model for the robot is illustrated in robot.png.

One aspect that needs to be pointed out is that, for this particular project, my focus was not robotics itself, it was the use of C language. Hence, some practical things were intentionally neglected, such as mechanical limits, the orientation of the effectuator, among others.

