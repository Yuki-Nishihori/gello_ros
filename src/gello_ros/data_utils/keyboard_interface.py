import asyncio

class KBInterface:
    def __init__(self):
        self.state = 'pass'

    def update(self):
        """
        状態を更新
        """
        return self.state

    async def set_start(self):
        """
        非同期で 'start'状態に設定
        """
        self.state = 'start'
        self.update()
        await asyncio.sleep(0.1)
        print("Start state set.")

    async def set_pass(self):
        """
        非同期で 'pass'状態に設定
        """
        self.state = 'pass'
        self.update()
        await asyncio.sleep(0.1)
        print("Pass state set.")

    async def set_quit(self):
        """
        非同期で 'quit'状態に設定し、終了する
        """
        self.state = 'quit'
        self.update()
        await asyncio.sleep(0.1)
        print("Quit state set.")
        return True  # 終了を示すためにTrueを返す

    async def handle_input(self):
        """
        ユーザー入力を非同期で受け付けて状態を変更
        """
        while True:
            user_input = input("Enter command (start/pass/quit): ").strip().lower()

            if user_input == "start":
                await self.set_start()
            elif user_input == "pass":
                await self.set_pass()
            elif user_input == "quit":
                if await self.set_quit():
                    break
            else:
                print("Invalid command. Please enter 'start', 'pass', or 'quit'.")

    async def run(self):
        """
        非同期でターミナル入力を処理
        """
        await self.handle_input()


# 外部から呼び出す例
if __name__ == "__main__":
    kb_interface = KBInterface()
    asyncio.run(kb_interface.run())
