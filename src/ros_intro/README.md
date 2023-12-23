# ROS Intro
Welcome to the very first lesson of UoZ Robotics: How to use ROS.

## What is ROS?
From [ros.org](https://www.ros.org/):

> The Robot Operating System (ROS) is a set of software libraries and tools that help you build robot applications. From drivers to state-of-the-art algorithms, and with powerful developer tools, ROS has what you need for your next robotics project. And it's all open source.

In more detail, the libraries and tools from ROS enable you to:
- Write multiple processes in different languages that can communicate with each other.
- Easily integrate open source code from around the world that implements algorithms from both industry and cutting edge academics
- Analyse, visualize and debug your robotics system with tools that are catered to the types of probems roboticists need to solve.

## How ROS works

### Pub/Sub Communication
ROS is primarily a set of libraries for Python and C++, and at the core of it's utility is it's [pub/sub interprocess communication system](https://aws.amazon.com/what-is/pub-sub-messaging/). In ROS this works with the following concepts:
- Node: A node is the name of a process that is using ROS libraries for communication.
- Message: A message is a package of information that can be interpreted by the ROS libraries.
- Publisher: A publisher is a node that is making messages available for other nodes to receive.
- Subscriber: A subscriber is a node that is receiving messages from a publishing node.
- Topic: This is what connects the publisher and subscriber. The publisher publishes messages to a topic, and when the subscriber subscribes to the same topic, the messages start flowing from the publisher to the subscriber.

### Services
One thing to note about pub/sub communication is that it is unidirectional. The publisher does not know about who is subscribing to them and receives no feedback. This makes it very efficient for sending data at high frequencies.

Sometimes, we want to ask for information from another node and receive it back. For this ROS uses a concept called services.

A service functions very similarly to a pub/sub set up, it has the following components:
- Node
- Service Definition: This is like a two-part message, it defines a Request message and a Response message
- Service Name: This is the equivalent to a topic.
- Service Server: This is the node that is ready to take requests and give back responses
- Service Client: This is any node that will be requesting a response from the server.

A service server will advertise it's service with the service name - think of this like an address. Then a client will send a request using the service name, and receive back a response. In a sense it is like calling a function that is running in a different process.

You should read more on ROS concepts [here](http://wiki.ros.org/ROS/Concepts).

## Assignments
### Assignment 1
1. Write a publisher and subscriber in Python following this tutorial: http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29
    1. You can replace beginner_tutorials with ros_intro
2. Write a publisher and subscriber in C++ following this tutorial: http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
    1. You can replace beginner_tutorials with ros_intro
3. Change your nodes so that you can run all four at once, that the Python subscriber is receiving messages from the C++ publisher, and that the C++ subscriber is receiving messages from the Python publisher.
4. Use [rqt_graph](http://wiki.ros.org/rqt_graph) to bring up a graph of your system, take a screenshot and put it in the results folder.

