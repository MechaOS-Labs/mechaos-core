import rclnodejs from "rclnodejs";
import EventEmitter from "events";

export class ROS2Client extends EventEmitter {
  constructor(config, logger) {
    super();
    this.config = config;
    this.logger = logger;
    this.node = null;
    this.publishers = {};
    this.subscribers = {};
    this.services = {};
    this.connected = false;
    this.heartbeatInterval = null;
    this.robotRegistry = new Map();

    this.topics = {
      TASKS: "/mechaos/tasks",
      TASK_COMPLETIONS: "/mechaos/task_completions",
      ROBOT_STATUS: "/mechaos/robot_status",
      ROBOT_HEARTBEAT: "/mechaos/robot_heartbeat",
      BRIDGE_STATUS: "/mechaos/bridge_status",
    };
  }

  async initialize() {
    try {
      this.logger.info("🤖 Initializing ROS2 client...");

      await rclnodejs.init({
        domain_id: this.config.domainId,
      });

      this.node = rclnodejs.createNode(this.config.nodeName);
      this.logger.info(`📡 Created ROS2 node: ${this.config.nodeName}`);

      await this.setupPublishers();

      await this.setupSubscribers();

      await this.setupServices();

      this.startSpinning();

      this.startHeartbeat();

      this.connected = true;
      this.emit("connected");
      this.logger.info("✅ ROS2 client initialized successfully");
    } catch (error) {
      this.logger.error("❌ Failed to initialize ROS2 client:", error);
      throw error;
    }
  }

  async setupPublishers() {
    try {
      this.publishers.tasks = this.node.createPublisher(
        "mechaos_msgs/Task",
        this.topics.TASKS,
        { qos: rclnodejs.QoS.profileDefaultQos }
      );

      this.publishers.bridgeStatus = this.node.createPublisher(
        "mechaos_msgs/BridgeStatus",
        this.topics.BRIDGE_STATUS,
        { qos: rclnodejs.QoS.profileDefaultQos }
      );

      this.logger.info("📤 Publishers setup complete");
    } catch (error) {
      this.logger.error("❌ Failed to setup publishers:", error);
      throw error;
    }
  }

  async setupSubscribers() {
    try {
      this.subscribers.taskCompletions = this.node.createSubscription(
        "mechaos_msgs/TaskCompletion",
        this.topics.TASK_COMPLETIONS,
        { qos: rclnodejs.QoS.profileDefaultQos },
        (msg) => this.handleTaskCompletion(msg)
      );

      this.subscribers.robotStatus = this.node.createSubscription(
        "mechaos_msgs/RobotStatus",
        this.topics.ROBOT_STATUS,
        { qos: rclnodejs.QoS.profileDefaultQos },
        (msg) => this.handleRobotStatus(msg)
      );

      this.subscribers.robotHeartbeat = this.node.createSubscription(
        "mechaos_msgs/RobotHeartbeat",
        this.topics.ROBOT_HEARTBEAT,
        { qos: rclnodejs.QoS.profileDefaultQos },
        (msg) => this.handleRobotHeartbeat(msg)
      );

      this.logger.info("📥 Subscribers setup complete");
    } catch (error) {
      this.logger.error("❌ Failed to setup subscribers:", error);
      throw error;
    }
  }

  async setupServices() {
    try {
      this.services.registerRobot = this.node.createService(
        "mechaos_msgs/srv/RegisterRobot",
        "/mechaos/register_robot",
        (request, response) => this.handleRobotRegistration(request, response)
      );

      this.services.assignTask = this.node.createService(
        "mechaos_msgs/srv/AssignTask",
        "/mechaos/assign_task",
        (request, response) => this.handleTaskAssignment(request, response)
      );

      this.logger.info("🔧 Services setup complete");
    } catch (error) {
      this.logger.error("❌ Failed to setup services:", error);
      throw error;
    }
  }

  startSpinning() {
    rclnodejs.spin(this.node);
    this.logger.info("🔄 ROS2 node spinning started");
  }

  startHeartbeat() {
    this.heartbeatInterval = setInterval(() => {
      this.publishBridgeStatus();
    }, 5000);

    this.logger.info("💓 Bridge heartbeat started");
  }

  publishBridgeStatus() {
    try {
      if (!this.publishers.bridgeStatus) {
        return;
      }

      const statusMsg = {
        node_name: this.config.nodeName,
        timestamp: Math.floor(Date.now() / 1000),
        connected_robots: this.robotRegistry.size,
        active_tasks: 0,
        ethereum_connected: true,
        ipfs_connected: true,
        uptime: process.uptime(),
        memory_usage: process.memoryUsage().heapUsed / 1024 / 1024,
        cpu_usage: process.cpuUsage(),
      };

      this.publishers.bridgeStatus.publish(statusMsg);
    } catch (error) {
      this.logger.error("❌ Failed to publish bridge status:", error);
    }
  }

  handleTaskCompletion(msg) {
    try {
      const completionData = {
        taskId: msg.task_id,
        robotId: msg.robot_id,
        completionTime: new Date(msg.completion_time * 1000),
        success: msg.success,
        proofHash: msg.proof_hash,
        telemetryData: msg.telemetry_data,
        errorMessage: msg.error_message || null,
        executionTime: msg.execution_time || 0,
        batteryLevel: msg.battery_level || null,
        position: msg.position || null,
      };

      this.logger.info(
        `✅ Task completion received: ${msg.task_id} by ${msg.robot_id} (Success: ${msg.success})`
      );
      this.emit("taskCompletion", completionData);
    } catch (error) {
      this.logger.error("❌ Failed to handle task completion:", error);
    }
  }

  handleRobotStatus(msg) {
    try {
      const statusData = {
        robotId: msg.robot_id,
        status: msg.status,
        batteryLevel: msg.battery_level,
        position: {
          x: msg.position.x,
          y: msg.position.y,
          z: msg.position.z,
          frame_id: msg.position.frame_id,
        },
        currentTask: msg.current_task || null,
        capabilities: msg.capabilities || [],
        lastUpdate: new Date(msg.timestamp * 1000),
        errorMessage: msg.error_message || null,
      };

      this.robotRegistry.set(msg.robot_id, statusData);

      this.logger.debug(
        `🤖 Robot status update: ${msg.robot_id} - ${msg.status}`
      );
      this.emit("robotStatus", statusData);
    } catch (error) {
      this.logger.error("❌ Failed to handle robot status:", error);
    }
  }

  handleRobotHeartbeat(msg) {
    try {
      const heartbeatData = {
        robotId: msg.robot_id,
        timestamp: new Date(msg.timestamp * 1000),
        batteryLevel: msg.battery_level,
        status: msg.status,
        uptime: msg.uptime,
      };

      if (this.robotRegistry.has(msg.robot_id)) {
        const robot = this.robotRegistry.get(msg.robot_id);
        robot.lastSeen = heartbeatData.timestamp;
        robot.batteryLevel = heartbeatData.batteryLevel;
        this.robotRegistry.set(msg.robot_id, robot);
      }

      this.emit("robotHeartbeat", heartbeatData);
    } catch (error) {
      this.logger.error("❌ Failed to handle robot heartbeat:", error);
    }
  }

  handleRobotRegistration(request, response) {
    try {
      const robotData = {
        robotId: request.robot_id,
        name: request.name,
        capabilities: request.capabilities,
        position: request.position,
        registrationTime: new Date(),
      };

      this.robotRegistry.set(request.robot_id, {
        ...robotData,
        status: "idle",
        lastSeen: new Date(),
        batteryLevel: 100,
      });

      response.success = true;
      response.message = `Robot ${request.robot_id} registered successfully`;

      this.logger.info(
        `🤖 Robot registered: ${request.name} (${request.robot_id})`
      );
      this.emit("robotRegistered", robotData);

      return response;
    } catch (error) {
      this.logger.error("❌ Failed to handle robot registration:", error);
      response.success = false;
      response.message = `Registration failed: ${error.message}`;
      return response;
    }
  }

  handleTaskAssignment(request, response) {
    try {
      const robotId = request.robot_id;
      const taskId = request.task_id;

      if (!this.robotRegistry.has(robotId)) {
        response.success = false;
        response.message = `Robot ${robotId} not found`;
        return response;
      }

      const robot = this.robotRegistry.get(robotId);
      if (robot.status !== "idle") {
        response.success = false;
        response.message = `Robot ${robotId} is not available (status: ${robot.status})`;
        return response;
      }

      robot.status = "assigned";
      robot.currentTask = taskId;
      this.robotRegistry.set(robotId, robot);

      response.success = true;
      response.message = `Task ${taskId} assigned to robot ${robotId}`;

      this.logger.info(`📋 Task assigned: ${taskId} → ${robotId}`);
      this.emit("taskAssigned", { taskId, robotId });

      return response;
    } catch (error) {
      this.logger.error("❌ Failed to handle task assignment:", error);
      response.success = false;
      response.message = `Assignment failed: ${error.message}`;
      return response;
    }
  }

  getConnectedRobots() {
    const now = new Date();
    const fiveMinutesAgo = new Date(now.getTime() - 5 * 60 * 1000);

    return Array.from(this.robotRegistry.values()).filter(
      (robot) => robot.lastSeen > fiveMinutesAgo
    );
  }

  getRobotById(robotId) {
    return this.robotRegistry.get(robotId) || null;
  }

  getRobotsByCapability(capability) {
    return Array.from(this.robotRegistry.values()).filter(
      (robot) => robot.capabilities && robot.capabilities.includes(capability)
    );
  }

  getAvailableRobots() {
    return Array.from(this.robotRegistry.values()).filter(
      (robot) => robot.status === "idle"
    );
  }

  async callService(serviceName, serviceType, request) {
    try {
      const client = this.node.createClient(serviceType, serviceName);

      const isServiceReady = await client.waitForService(5000);
      if (!isServiceReady) {
        throw new Error(`Service ${serviceName} not available`);
      }

      const response = await client.sendRequest(request);
      client.destroy();

      return response;
    } catch (error) {
      this.logger.error(`❌ Failed to call service ${serviceName}:`, error);
      throw error;
    }
  }

  async publishTopicOnce(topicName, messageType, message) {
    try {
      const publisher = this.node.createPublisher(messageType, topicName);

      await new Promise((resolve) => setTimeout(resolve, 100));

      publisher.publish(message);
      publisher.destroy();

      this.logger.info(`📤 Published to ${topicName}`);
    } catch (error) {
      this.logger.error(`❌ Failed to publish to ${topicName}:`, error);
      throw error;
    }
  }

  isConnected() {
    return this.connected && this.node !== null;
  }

  async close() {
    this.logger.info("🔌 Closing ROS2 client...");

    if (this.heartbeatInterval) {
      clearInterval(this.heartbeatInterval);
      this.heartbeatInterval = null;
    }

    Object.values(this.publishers).forEach((publisher) => {
      if (publisher) publisher.destroy();
    });
    this.publishers = {};

    Object.values(this.subscribers).forEach((subscriber) => {
      if (subscriber) subscriber.destroy();
    });
    this.subscribers = {};

    Object.values(this.services).forEach((service) => {
      if (service) service.destroy();
    });
    this.services = {};

    if (this.node) {
      this.node.destroy();
      this.node = null;
    }

    try {
      await rclnodejs.shutdown();
    } catch (error) {
      this.logger.warn("⚠️ Error during ROS2 shutdown:", error);
    }

    this.connected = false;
    this.robotRegistry.clear();

    this.emit("closed");
    this.logger.info("✅ ROS2 client closed");
  }
}
