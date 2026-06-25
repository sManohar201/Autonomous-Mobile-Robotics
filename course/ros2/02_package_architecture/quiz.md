# ROS2 Module 02 Quiz - Package Architecture

## Q1

Why should interface packages avoid depending on application packages?

<details><summary>Answer</summary>

Interfaces are low-level contracts. If they depend on applications, dependency direction becomes circular and every interface change risks pulling in unrelated runtime code.

</details>

## Q2

What belongs in a bringup package?

<details><summary>Answer</summary>

Top-level launch files, robot/environment selection, composition decisions, parameter file selection, and orchestration of subsystem packages.

</details>

## Q3

Why separate `robot_description` from application logic?

<details><summary>Answer</summary>

Robot description assets are shared by simulation, visualization, planning, and bringup. Keeping them separate avoids coupling model files to application code.

</details>

## Q4

Where should custom messages live?

<details><summary>Answer</summary>

In a dedicated interface package, commonly named `*_interfaces`, so multiple packages can depend on the same stable message/service/action contracts.

</details>

## Q5

What is the risk of putting every node into one package?

<details><summary>Answer</summary>

It hides ownership boundaries, increases rebuild scope, encourages tangled dependencies, and makes reuse/testing harder.

</details>

