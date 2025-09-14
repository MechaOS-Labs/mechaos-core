#!/usr/bin/env node
const { ethers } = require("ethers");
const fs = require("fs");

async function deployContracts() {
  const provider = new ethers.JsonRpcProvider(process.env.RPC_URL);
  const wallet = new ethers.Wallet(process.env.PRIVATE_KEY, provider);

  const contractJson = JSON.parse(
    fs.readFileSync("../contracts/build/TaskManager.json")
  );

  const factory = new ethers.ContractFactory(
    contractJson.abi,
    contractJson.bytecode,
    wallet
  );

  console.log("Deploying TaskManager contract...");
  const contract = await factory.deploy();
  await contract.waitForDeployment();

  console.log("Contract deployed to:", await contract.getAddress());

  const deploymentInfo = {
    address: await contract.getAddress(),
    abi: contractJson.abi,
    network: await provider.getNetwork(),
    deployer: wallet.address,
  };

  fs.writeFileSync("deployment.json", JSON.stringify(deploymentInfo, null, 2));
}

deployContracts().catch(console.error);
