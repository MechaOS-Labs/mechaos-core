#!/usr/bin/env node
import dotenv from "dotenv";
import express from "express";
import cors from "cors";
import helmet from "helmet";
import { createServer } from "http";
import winston from "winston";

import { EthereumClient } from "./ethereum/EthereumClient.js";
import { ROS2Client } from "./ros2/ROS2Client.js";
import { IPFSClient } from "./ipfs/IPFSClient.js";
import { WebSocketServer } from "./websocket/WebSocketServer.js";
import { BridgeCore } from "./core/BridgeCore.js";
import { HealthMonitor } from "./utils/HealthMonitor.js";
import { setupRoutes } from "./api/routes.js";

dotenv.config();

const logger = winston.createLogger({
  level: process.env.LOG_LEVEL || "info",
  format: winston.format.combine(
    winston.format.timestamp(),
    winston.format.errors({ stack: true }),
    winston.format.json()
  ),
  transports: [
    new winston.transports.Console({
      format: winston.format.combine(
        winston.format.colorize(),
        winston.format.simple()
      ),
    }),
    new winston.transports.File({ filename: "logs/bridge.log" }),
  ],
});

class MechaOSBridge {
  constructor() {
    this.app = express();
    this.server = createServer(this.app);
    this.clients = {};
    this.bridgeCore = null;
    this.healthMonitor = null;
  }

  async initialize() {
    try {
      logger.info("🤖 Starting MechaOS Bridge...");

      this.setupMiddleware();

      await this.initializeClients();

      this.bridgeCore = new BridgeCore(this.clients, logger);
      await this.bridgeCore.initialize();

      setupRoutes(this.app, this.bridgeCore, logger);

      this.clients.websocket = new WebSocketServer(
        this.server,
        this.bridgeCore,
        logger
      );
      await this.clients.websocket.initialize();

      this.healthMonitor = new HealthMonitor(this.clients, logger);
      this.healthMonitor.start();

      const port = process.env.BRIDGE_PORT || 3001;
      this.server.listen(port, () => {
        logger.info(`🌉 MechaOS Bridge running on port ${port}`);
        logger.info("🔗 Bridge components initialized successfully");
        this.logSystemStatus();
      });
    } catch (error) {
      logger.error("❌ Failed to initialize MechaOS Bridge:", error);
      process.exit(1);
    }
  }

  setupMiddleware() {
    this.app.use(helmet());
    this.app.use(
      cors({
        origin: process.env.ALLOWED_ORIGINS?.split(",") || [
          "http://localhost:3000",
        ],
        credentials: true,
      })
    );
    this.app.use(express.json({ limit: "10mb" }));
    this.app.use(express.urlencoded({ extended: true }));

    // Request logging
    this.app.use((req, res, next) => {
      logger.info(`${req.method} ${req.path} - ${req.ip}`);
      next();
    });
  }

  async initializeClients() {
    logger.info("🔧 Initializing bridge clients...");

    this.clients.ethereum = new EthereumClient(
      {
        rpcUrl: process.env.ETHEREUM_RPC_URL || "http://localhost:8545",
        contractAddress: process.env.CONTRACT_ADDRESS,
        networkId: process.env.ETHEREUM_NETWORK_ID || "1337",
      },
      logger
    );
    await this.clients.ethereum.initialize();

    this.clients.ros2 = new ROS2Client(
      {
        domainId: process.env.ROS_DOMAIN_ID || 42,
        nodeName: "mechaos_bridge",
      },
      logger
    );
    await this.clients.ros2.initialize();

    this.clients.ipfs = new IPFSClient(
      {
        apiUrl: process.env.IPFS_API_URL || "http://localhost:5001",
        gatewayUrl: process.env.IPFS_GATEWAY_URL || "http://localhost:8080",
      },
      logger
    );
    await this.clients.ipfs.initialize();

    logger.info("✅ All clients initialized successfully");
  }

  logSystemStatus() {
    logger.info("📊 System Status:");
    logger.info(
      `   • Ethereum: ${
        this.clients.ethereum.isConnected() ? "🟢 Connected" : "🔴 Disconnected"
      }`
    );
    logger.info(
      `   • ROS2: ${
        this.clients.ros2.isConnected() ? "🟢 Connected" : "🔴 Disconnected"
      }`
    );
    logger.info(
      `   • IPFS: ${
        this.clients.ipfs.isConnected() ? "🟢 Connected" : "🔴 Disconnected"
      }`
    );
    logger.info(
      `   • WebSocket: ${this.clients.websocket ? "🟢 Active" : "🔴 Inactive"}`
    );
  }

  async shutdown() {
    logger.info("🛑 Shutting down MechaOS Bridge...");

    if (this.healthMonitor) {
      this.healthMonitor.stop();
    }

    if (this.clients.ethereum) {
      await this.clients.ethereum.close();
    }
    if (this.clients.ros2) {
      await this.clients.ros2.close();
    }
    if (this.clients.ipfs) {
      await this.clients.ipfs.close();
    }
    if (this.clients.websocket) {
      await this.clients.websocket.close();
    }

    if (this.server) {
      this.server.close();
    }

    logger.info("👋 MechaOS Bridge shutdown complete");
    process.exit(0);
  }
}

const bridge = new MechaOSBridge();

process.on("SIGTERM", () => bridge.shutdown());
process.on("SIGINT", () => bridge.shutdown());
process.on("uncaughtException", (error) => {
  logger.error("Uncaught Exception:", error);
  bridge.shutdown();
});
process.on("unhandledRejection", (reason, promise) => {
  logger.error("Unhandled Rejection at:", promise, "reason:", reason);
  bridge.shutdown();
});

bridge.initialize().catch((error) => {
  logger.error("Failed to start bridge:", error);
  process.exit(1);
});
