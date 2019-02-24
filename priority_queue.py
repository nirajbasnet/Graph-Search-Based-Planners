#!/usr/bin/env python3

""" Priority Queue Module

This module contains the PriorityQueue class converted from Python.

Author: Scott Chow
Code based on https://docs.python.org/3/library/heapq.html
Function Conventions follow Matlab code provided by Geoffrey Hollinger
"""

import itertools
from heapq import heappop, heappush


class PriorityQueue():
    """ An implementation of Priority Queue using heapq

    Notes:
        Taken from: https://docs.python.org/3/library/heapq.html
        Items inserted into the priority queue must be hashable (e.g. tuples, numbers, strings)
    """
    def __init__(self):
        """
            Creates an empty priority queue
        """
        # Priority queue arranged in a heap
        self.pq = []
        # Map items to entries
        self.entry_finder = {}
        # Counter for unique sequence
        self.counter = itertools.count()
        # Placeholder for removed items
        self.REMOVED = '<removed-task>'
        # Number of valid items in the queue
        self.size = 0

    def __len__(self):
        return self.size

    def insert(self, item, priority=0):
        """ Add a new item or update priority of existing item

        Args:
            item (any type with __str__ defined): item to be placed into a queue
            priority (optional, float): the priority of the item

        Notes:
            This corresponds to the set() function in MATLAB.
            Renamed because set is a type in Python
        """
        if item in self.entry_finder:
            self._remove_item(item)

        count = next(self.counter)
        entry = [priority, count, item]
        self.entry_finder[item] = entry
        heappush(self.pq, entry)
        self.size += 1

    def _remove_item(self, item):
        """ Mark an existing item as REMOVED. Raise KeyError if not found. 

        Args:
            item: the item to be removed
        """
        entry = self.entry_finder[item]
        entry[-1] = self.REMOVED
        self.size -= 1

    def pop(self):
        """ Remove and return lowest priority item """
        while self.pq:
            _, _, item = heappop(self.pq)
            if item is not self.REMOVED:
                del self.entry_finder[item]
                self.size -= 1
                return item
        raise KeyError('Pop from empty priority queue')

    def test(self, item):
        """ Checks if item is in the priority queue

        Args:
            item: item to check
        """
        if item in self.entry_finder:
            return self.entry_finder[item] is not self.REMOVED
        else:
            return False
