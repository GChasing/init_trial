import threading
import time
import logging

def test_thread(para='hi', sleep=3):
    """线程运行函数"""
    time.sleep(sleep)
    print(para)

def main():
    thread_hi = threading.Thread(target=test_thread)
    thread_hello = threading.Thread(target=test_thread, args=('hello', 1))
    # 启动线程
    thread_hi.start()
    thread_hello.start()
    print('Main thread has ended!')
    logging.info("This is logging info")
    logging.warning("This is logging warn")
    logging.error("This is logging error")

if __name__ == '__main__':
    main()
