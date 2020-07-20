# -*- coding:utf-8 -*-
from abc import ABCMeta, abstractmethod

class ext_pos_source:
    __metaclass__ = ABCMeta
    def __init__(self, param):
        pass

    @abstractmethod
    def update_position(self, pos_ext):
        return pos_ext