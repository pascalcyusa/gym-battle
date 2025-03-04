import requests
from dotenv import load_dotenv
import os

load_dotenv()

ACCESS_TOKEN = os.getenv('AIRTABLE_ACCESS_TOKEN')
BASE_ID = os.getenv('AIRTABLE_BASE_ID')
TABLE_NAME = "gym-battle"
URL = f"https://api.airtable.com/v0/{BASE_ID}/Table%201"

headers = {
    "Authorization": f"Bearer {ACCESS_TOKEN}",
    "Content-Type": "application/json"
}

def get_commands():
    response = requests.get(URL, headers=headers)
    if response.status_code == 200:
        return response.json().get("records", [])
    else:
        print(f"Error: {response.status_code} - {response.text}")
        return []

def add_command(action, parameters):
    data = {
        "fields": {
            "Action": action,
            "Parameters": parameters
        }
    }
    response = requests.post(URL, json=data, headers=headers)
    if response.status_code == 200:
        print("Command added successfully.")
    else:
        print(f"Error: {response.status_code} - {response.text}")
