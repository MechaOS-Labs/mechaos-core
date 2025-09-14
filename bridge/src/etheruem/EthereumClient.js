import { ethers } from "ethers";
import EventEmitter from "events";

export class EthereumClient extends EventEmitter {
  constructor(config, logger) {
    super();
    this.config = config;
    this.logger = logger;
    this.provider = null;
    this.contract = null;
    this.wallet = null;
    this.connected = false;
    this.reconnectAttempts = 0;
    this.maxReconnectAttempts = 5;
    this.taskListeners = new Map();

    this.contractABI = [
      "event TaskCreated(uint256 indexed taskId, address indexed creator, string taskType, uint256 reward, string description, string location)",
      "event TaskClaimed(uint256 indexed taskId, address indexed robot)",
      "event TaskCompleted(uint256 indexed taskId, address indexed robot, string proofHash)",
      "event RobotRegistered(address indexed robot, string name, string[] capabilities)",

      "function createTask(string memory taskType, string memory description, string memory location) external payable returns (uint256)",
      "function claimTask(uint256 taskId) external",
      "function submitProof(uint256 taskId, string memory proofHash) external",
      "function registerRobot(string memory name, string[] memory capabilities, string memory location) external",
      "function getTask(uint256 taskId) external view returns (tuple(uint256 id, address creator, address assignedRobot, string taskType, string description, string location, uint256 reward, uint8 status, string proofHash, uint256 createdAt, uint256 completedAt))",
      "function getRobot(address robotAddress) external view returns (tuple(address robotAddress, string name, string[] capabilities, string location, bool active, uint256 registeredAt, uint256 tasksCompleted))",
      "function getTaskCount() external view returns (uint256)",
    ];
  }

  async initialize() {
    try {
      this.logger.info("⚡ Initializing Ethereum client...");

      this.provider = new ethers.JsonRpcProvider(this.config.rpcUrl);

      const network = await this.provider.getNetwork();
      this.logger.info(
        `🔗 Connected to Ethereum network: ${network.name} (Chain ID: ${network.chainId})`
      );

      if (this.config.contractAddress) {
        this.contract = new ethers.Contract(
          this.config.contractAddress,
          this.contractABI,
          this.provider
        );

        const code = await this.provider.getCode(this.config.contractAddress);
        if (code === "0x") {
          throw new Error(
            `No contract found at address: ${this.config.contractAddress}`
          );
        }

        this.logger.info(`📋 Contract loaded: ${this.config.contractAddress}`);

        await this.setupEventListeners();
      } else {
        this.logger.warn(
          "⚠️  No contract address provided, contract interactions disabled"
        );
      }

      if (process.env.BRIDGE_PRIVATE_KEY) {
        this.wallet = new ethers.Wallet(
          process.env.BRIDGE_PRIVATE_KEY,
          this.provider
        );
        this.logger.info(
          `💳 Bridge wallet initialized: ${this.wallet.address}`
        );
      }

      this.connected = true;
      this.emit("connected");
      this.logger.info("✅ Ethereum client initialized successfully");
    } catch (error) {
      this.logger.error("❌ Failed to initialize Ethereum client:", error);
      await this.handleConnectionError(error);
    }
  }

  async setupEventListeners() {
    if (!this.contract) return;

    try {
      this.contract.on(
        "TaskCreated",
        (taskId, creator, taskType, reward, description, location, event) => {
          const taskData = {
            taskId: taskId.toString(),
            creator,
            taskType,
            reward: ethers.formatEther(reward),
            description,
            location,
            blockNumber: event.log.blockNumber,
            txHash: event.log.transactionHash,
          };

          this.logger.info(`📋 New task created: ${taskId} (${taskType})`);
          this.emit("taskCreated", taskData);
        }
      );

      this.contract.on("TaskClaimed", (taskId, robot, event) => {
        const claimData = {
          taskId: taskId.toString(),
          robot,
          blockNumber: event.log.blockNumber,
          txHash: event.log.transactionHash,
        };

        this.logger.info(`🤖 Task claimed: ${taskId} by ${robot}`);
        this.emit("taskClaimed", claimData);
      });

      this.contract.on("TaskCompleted", (taskId, robot, proofHash, event) => {
        const completionData = {
          taskId: taskId.toString(),
          robot,
          proofHash,
          blockNumber: event.log.blockNumber,
          txHash: event.log.transactionHash,
        };

        this.logger.info(`✅ Task completed: ${taskId} by ${robot}`);
        this.emit("taskCompleted", completionData);
      });

      this.contract.on(
        "RobotRegistered",
        (robot, name, capabilities, event) => {
          const registrationData = {
            robot,
            name,
            capabilities,
            blockNumber: event.log.blockNumber,
            txHash: event.log.transactionHash,
          };

          this.logger.info(`🤖 Robot registered: ${name} (${robot})`);
          this.emit("robotRegistered", registrationData);
        }
      );

      this.logger.info("👂 Event listeners setup complete");
    } catch (error) {
      this.logger.error("❌ Failed to setup event listeners:", error);
      throw error;
    }
  }

  async getTask(taskId) {
    if (!this.contract) {
      throw new Error("Contract not initialized");
    }

    try {
      const task = await this.contract.getTask(taskId);
      return {
        id: task.id.toString(),
        creator: task.creator,
        assignedRobot: task.assignedRobot,
        taskType: task.taskType,
        description: task.description,
        location: task.location,
        reward: ethers.formatEther(task.reward),
        status: task.status,
        proofHash: task.proofHash,
        createdAt: new Date(Number(task.createdAt) * 1000),
        completedAt:
          task.completedAt > 0
            ? new Date(Number(task.completedAt) * 1000)
            : null,
      };
    } catch (error) {
      this.logger.error(`❌ Failed to get task ${taskId}:`, error);
      throw error;
    }
  }

  async getRobot(robotAddress) {
    if (!this.contract) {
      throw new Error("Contract not initialized");
    }

    try {
      const robot = await this.contract.getRobot(robotAddress);
      return {
        address: robot.robotAddress,
        name: robot.name,
        capabilities: robot.capabilities,
        location: robot.location,
        active: robot.active,
        registeredAt: new Date(Number(robot.registeredAt) * 1000),
        tasksCompleted: Number(robot.tasksCompleted),
      };
    } catch (error) {
      this.logger.error(`❌ Failed to get robot ${robotAddress}:`, error);
      throw error;
    }
  }

  async getTaskCount() {
    if (!this.contract) {
      throw new Error("Contract not initialized");
    }

    try {
      const count = await this.contract.getTaskCount();
      return Number(count);
    } catch (error) {
      this.logger.error("❌ Failed to get task count:", error);
      throw error;
    }
  }

  async createTask(taskData, signerPrivateKey) {
    if (!this.contract) {
      throw new Error("Contract not initialized");
    }

    try {
      const signer = new ethers.Wallet(signerPrivateKey, this.provider);
      const contractWithSigner = this.contract.connect(signer);

      const tx = await contractWithSigner.createTask(
        taskData.taskType,
        taskData.description,
        taskData.location,
        {
          value: ethers.parseEther(taskData.reward.toString()),
          gasLimit: 500000,
        }
      );

      this.logger.info(`📤 Task creation transaction sent: ${tx.hash}`);
      const receipt = await tx.wait();

      const taskCreatedEvent = receipt.logs.find(
        (log) =>
          log.topics[0] ===
          ethers.id("TaskCreated(uint256,address,string,uint256,string,string)")
      );

      if (taskCreatedEvent) {
        const taskId = ethers.getBigInt(taskCreatedEvent.topics[1]).toString();
        this.logger.info(`✅ Task created successfully with ID: ${taskId}`);
        return {
          taskId,
          txHash: receipt.hash,
          blockNumber: receipt.blockNumber,
        };
      }

      throw new Error("TaskCreated event not found in transaction receipt");
    } catch (error) {
      this.logger.error("❌ Failed to create task:", error);
      throw error;
    }
  }

  async submitProof(taskId, proofHash, robotPrivateKey) {
    if (!this.contract) {
      throw new Error("Contract not initialized");
    }

    try {
      const signer = new ethers.Wallet(robotPrivateKey, this.provider);
      const contractWithSigner = this.contract.connect(signer);

      const tx = await contractWithSigner.submitProof(taskId, proofHash, {
        gasLimit: 300000,
      });

      this.logger.info(`📤 Proof submission transaction sent: ${tx.hash}`);
      const receipt = await tx.wait();

      this.logger.info(`✅ Proof submitted successfully for task ${taskId}`);
      return { txHash: receipt.hash, blockNumber: receipt.blockNumber };
    } catch (error) {
      this.logger.error(`❌ Failed to submit proof for task ${taskId}:`, error);
      throw error;
    }
  }

  async handleConnectionError(error) {
    this.connected = false;
    this.emit("disconnected", error);

    if (this.reconnectAttempts < this.maxReconnectAttempts) {
      this.reconnectAttempts++;
      const delay = Math.pow(2, this.reconnectAttempts) * 1000; // Exponential backoff

      this.logger.warn(
        `🔄 Reconnecting to Ethereum in ${delay / 1000}s (attempt ${
          this.reconnectAttempts
        }/${this.maxReconnectAttempts})`
      );

      setTimeout(() => {
        this.initialize();
      }, delay);
    } else {
      this.logger.error("❌ Max reconnection attempts reached, giving up");
      this.emit("maxReconnectAttemptsReached");
    }
  }

  async getBlockNumber() {
    if (!this.provider) return null;

    try {
      return await this.provider.getBlockNumber();
    } catch (error) {
      this.logger.error("❌ Failed to get block number:", error);
      return null;
    }
  }

  async getBalance(address) {
    if (!this.provider) return null;

    try {
      const balance = await this.provider.getBalance(address);
      return ethers.formatEther(balance);
    } catch (error) {
      this.logger.error(`❌ Failed to get balance for ${address}:`, error);
      return null;
    }
  }

  isConnected() {
    return this.connected && this.provider !== null;
  }

  async close() {
    this.logger.info("🔌 Closing Ethereum client...");

    if (this.contract) {
      this.contract.removeAllListeners();
    }

    this.connected = false;
    this.provider = null;
    this.contract = null;
    this.wallet = null;

    this.emit("closed");
    this.logger.info("✅ Ethereum client closed");
  }
}
