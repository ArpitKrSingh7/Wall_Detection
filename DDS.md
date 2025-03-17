# Why DDS in ROS 2?

By now, we know how ROS 1 and ROS 2 differ, especially when it comes to communication. One big question is: **why switch to DDS?** Why didn’t we just improve ROS 1’s existing transport layer? Or maybe even build our own middleware from scratch?

Let’s walk through it.

## Why not improve ROS 1's transport?

ROS 1 used a custom TCP-based protocol for message passing. It got the job done, but had some major limitations:

- **Real-time support was weak.** TCP isn’t great when you need deterministic, low-latency communication.
- **No fine-grained control.** Things like QoS (Quality of Service) policies—reliability, durability, deadline settings—were basically non-existent.
- **Manual discovery.** Nodes didn’t auto-discover; everything was centrally managed.
- **Scaling pain points.** As systems grew larger and more distributed, ROS 1’s transport started showing cracks.

Sure, we *could’ve* tried patching it up, but realistically, it would have been a mess. Too much effort for too little gain.

---

## Why not build a new middleware from scratch?

Another idea we considered: let’s piece together a custom middleware using libraries like:

- **ZeroMQ** → for messaging
- **Protocol Buffers / Cap’n Proto** → for serialization
- **Zeroconf (Bonjour/Avahi)** → for discovery

Sounds tempting, right? But here’s the catch:

- **We’d have to solve every problem ourselves.**
  - Message delivery guarantees
  - Discovery protocols
  - Security layers
  - QoS handling
  - Multi-platform support
- **Years of testing & optimization.**
- **Risk of fragmentation.**

Basically, reinventing the wheel. And others have already built reliable, well-tested solutions.

---

## Why DDS?

During research, **DDS (Data Distribution Service)** emerged as the clear winner.

### What DDS brings to the table:
- **Publish-Subscribe model** → exactly what ROS needs.
- **Built-in QoS policies** → fine control over reliability, deadlines, durability, etc.
- **Automatic discovery** → no need for central master nodes.
- **Supports real-time systems** → can handle UDP, TCP, multicast.
  > *Don’t worry, we’ll dive deeper into TCP, UDP, multicast, and how DDS uses them later in this section.*
- **Scalable & flexible** → works on small embedded devices to large distributed systems.
- **Security features out-of-the-box.**
- **Multiple open-source & commercial implementations.**

Basically, DDS checked all the boxes we needed, saving us from reinventing complex networking protocols.

---

## TL;DR

- Improving ROS 1’s transport = patchwork, limited scalability.
- Building middleware from scratch = too much effort, redoing solved problems.
- DDS = mature, scalable, secure, designed for exactly what ROS 2 needs.

So instead of spending years building and testing our own system, we integrated DDS and focused on building better robotics tools.

---
