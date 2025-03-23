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




# Discovery in DDS

In ROS 1, we had a **ROS Master** that handled node discovery. Every node had to check in with the Master to register itself and learn about others. This made the **Master** a single point of failure—if it died, the whole system broke.

ROS 2, with DDS, **ditched the Master completely.**

### How?

DDS uses a **distributed discovery system**:
- No central authority.
- Each node (or participant*) automatically discovers others using built-in protocols.
- No need to manually manage who talks to whom.

This makes the system:
- **More scalable** (works fine with many nodes).
- **No single point of failure.**

But, of course, users still want to list nodes, topics, etc. So, ROS 2 provides its own clean API on top of DDS to avoid making you deal with DDS directly.

**Bonus:** DDS also supports adding custom metadata during discovery. ROS 2 can use this to store extra info (like topic types) without messing up the basic system.

---

## Visual: ROS 1 vs ROS 2 Discovery


---

# Publish-Subscribe Transport in DDS

ROS 1 used custom protocols:
- **TCPROS** → based on TCP*
- **UDPROS** → based on UDP*

These handled message passing between nodes.

**In ROS 2, DDS replaces all of that** with:
- **DDSI-RTPS (Real-Time Publish-Subscribe)** → standard protocol built exactly for publish/subscribe communication.

### The DDS Pub-Sub Model:
| ROS 1 Concept | DDS Equivalent           | What's new?                          |
|--------------|---------------------------|--------------------------------------|
| Node         | Graph Participant*        | The big boss, owns topics & entities |
| Topic        | Topic                     | Similar                              |
| Publisher    | Publisher + DataWriter*   | More control, extra config           |
| Subscriber   | Subscriber + DataReader*  | Same as above                        |

**Key feature:**  
DDS lets you configure **QoS (Quality of Service)** settings at each level:  
You can fine-tune reliability, durability, deadlines, etc.

BUT — all these extra layers (DataWriters, DataReaders) would confuse most users.  
That’s why ROS 2 **wraps this complexity inside the usual Node, Publisher, Subscriber interface.**

---

# Efficient Transport Alternatives

### 🚀 Let's talk SPEED!

### ROS 1 approach:
- Default: TCP loopback for local comms.
- If more speed needed → **Nodelets** were used.

### What are Nodelets?

Think of **Nodelets** as multiple nodes running inside **one single process**.  
Why?
- Normally, two separate processes need to copy/serialize data to talk.
- But inside the same process, you can just pass a pointer! (No copy, no serialization.)

**How?**
They used **`boost::shared_ptr`*** — which is like a smart pointer that keeps track of shared data.

Result:
- Super-fast communication inside the process.
- No unnecessary overhead.

---

### DDS optimization:

Most **DDS vendors*** are smart.  
They auto-optimize message passing:

| Case                               | How DDS handles it                                 |
|-----------------------------------|----------------------------------------------------|
| Same machine, diff processes      | Use **shared-memory** (zero-copy, fast)            |
| Across machines                   | Use **UDP** or **TCP** based wire protocols        |

This shared-memory optimization gives a huge speed boost because:
- DDS won’t waste time breaking messages into small UDP packets.
- Direct memory access = faster than any network send.

BUT here’s a catch:
- Not every DDS vendor does it the same way.
- And, ROS messages may still need conversion to DDS format, slowing things down.

---

### ROS 2 Solution:

To fix this:
- ROS 2 has its **own intra-process communication system**.
- Directly passes pointers between publishers/subscribers (no serialization, no DDS conversion).
- Similar to how **nodelets** worked in ROS 1!

So whether DDS optimizes or not, ROS 2 ensures **fast, zero-copy communication** within the same process.

---

## Visual: DDS Transport Optimization


---

# Terms Explained 📝

| Term                 | Explanation                                                                                                                                     |
|---------------------|-------------------------------------------------------------------------------------------------------------------------------------------------|
| **TCP (Transmission Control Protocol)**  | Connection-based protocol. Reliable, ordered, but slower due to connection overhead.                                           |
| **UDP (User Datagram Protocol)**         | Connectionless protocol. Faster, but less reliable (no guarantee all packets arrive).                                          |
| **Graph Participant (DDS)**              | A participant in DDS is like a "node" that manages publishers, subscribers, and topics.                                         |
| **DataWriter & DataReader (DDS)**        | DDS-specific entities responsible for actually writing to or reading from topics. Adds extra configurability/QoS options.       |
| **boost::shared_ptr (Nodelets)**         | A smart pointer in C++ that allows multiple parts of a program to share access to the same data without copying it.              |
| **DDS Vendors**                          | Companies or orgs providing DDS implementations (e.g., RTI Connext, Fast-DDS, Cyclone DDS). Each may have slightly different optimizations. |
| **QoS (Quality of Service)**             | DDS settings that control reliability, durability, deadlines, and other communication behaviors.                               |

---

# TL;DR

- ROS 2 switched to DDS for distributed discovery, reliability, and scalability.
- DDS replaces ROS 1’s custom TCP/UDP protocols with a standard, flexible pub-sub model.
- ROS 2 still hides DDS complexity behind familiar interfaces.
- Intra-process communication is handled super efficiently, using pointer-passing, similar to how nodelets worked in ROS 1.
- Terms like TCP, UDP, QoS, DataReaders, and shared-memory all contribute to making ROS 2 communication both **robust and fast**.

---

**Need me to add example code snippets to show how intra-process communication looks in ROS 2 compared to ROS 1 nodelets?**



