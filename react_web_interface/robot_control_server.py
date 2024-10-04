import asyncio
import websockets
import json


async def handle_connection(websocket, path):
    print("Client connected")
    try:
        async for message in websocket:
            data = json.loads(message)
            buttons = data.get('buttons', {})
            axes = data.get('axes', {})

            print(f"Received button states: {buttons}")
            print(f"Received axis states: {axes}")
    except websockets.ConnectionClosed:
        print("Client disconnected")


async def main():
    async with websockets.serve(handle_connection, "localhost", 8765):
        print("WebSocket server started on ws://localhost:8765")
        await asyncio.Future()  # Run forever


if __name__ == "__main__":
    asyncio.run(main())
