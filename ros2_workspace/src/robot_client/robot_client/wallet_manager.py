#!/usr/bin/env python3
from web3 import Web3
from eth_account import Account
import os
import json

class RobotWallet:
    def __init__(self, private_key, rpc_url, contract_address, contract_abi):
        self.w3 = Web3(Web3.HTTPProvider(rpc_url))
        self.account = Account.from_key(private_key)
        self.contract = self.w3.eth.contract(
            address=contract_address,
            abi=contract_abi
        )
    
    def claim_task(self, task_id):
        """Claim a task from the smart contract"""
        try:
            function = self.contract.functions.claimTask(task_id)
            transaction = function.build_transaction({
                'from': self.account.address,
                'nonce': self.w3.eth.get_transaction_count(self.account.address),
                'gas': 500000,
                'gasPrice': self.w3.eth.gas_price,
            })
            
            signed_txn = self.w3.eth.account.sign_transaction(
                transaction, 
                private_key=self.account.key
            )
            
            tx_hash = self.w3.eth.send_raw_transaction(signed_txn.rawTransaction)
            receipt = self.w3.eth.wait_for_transaction_receipt(tx_hash)
            
            return receipt.status == 1
            
        except Exception as e:
            print(f"Error claiming task: {e}")
            return False
    
    def submit_proof(self, task_id, ipfs_hash):
        """Submit proof of task completion"""
        try:
            function = self.contract.functions.submitProof(task_id, ipfs_hash)
            transaction = function.build_transaction({
                'from': self.account.address,
                'nonce': self.w3.eth.get_transaction_count(self.account.address),
                'gas': 500000,
                'gasPrice': self.w3.eth.gas_price,
            })
            
            signed_txn = self.w3.eth.account.sign_transaction(
                transaction,
                private_key=self.account.key
            )
            
            tx_hash = self.w3.eth.send_raw_transaction(signed_txn.rawTransaction)
            receipt = self.w3.eth.wait_for_transaction_receipt(tx_hash)
            
            return receipt.status == 1
            
        except Exception as e:
            print(f"Error submitting proof: {e}")
            return False