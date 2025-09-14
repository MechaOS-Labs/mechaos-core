#!/usr/bin/env node
const { ethers } = require("ethers");
const fs = require("fs");

async function setupRobot() {
  const config = JSON.parse(fs.readFileSync("robot_config.json", "utf8"));

  const provider = new ethers.JsonRpcProvider(config.ethereum.rpcUrl);
  const wallet = new ethers.Wallet(config.robot.privateKey, provider);
  const contract = new ethers.Contract(
    config.ethereum.contractAddress,
    config.ethereum.abi,
    wallet
  );

  try {
    console.log("Registering robot...");
    const tx = await contract.registerRobot(
      config.robot.name,
      config.robot.capabilities,
      config.robot.location
    );

    await tx.wait();
    console.log("Robot registered successfully!");
    console.log("Robot address:", wallet.address);
    console.log("Transaction hash:", tx.hash);
  } catch (error) {
    console.error("Registration failed:", error);
  }
}

setupRobot();
