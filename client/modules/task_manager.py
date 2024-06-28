import threading

from typing import List


class TaskManager(object):
    tasks: List[threading.Thread] = []
    def __init__(self):
        self.tasks = []

    def add_task(self, task: threading.Thread):
        self.tasks.append(task)
        task.start()

    def remove_task(self, task: threading.Thread):
        self.tasks.remove(task)


task_manager = TaskManager()