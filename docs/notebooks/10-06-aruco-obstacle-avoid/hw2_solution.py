# Problem 6: Create your own updateable PriorityQueueUpdatable and use it reimplement dijkstra function
# Make all necessary changes
import math

# ======== Modified code from heapq.py ====================================
# 'heap' is a valid heap at all indices >= startpos, except possibly for pos.
# pos is the index of a leaf with a possibly out-of-order value.  Restore the
# heap invariant.
def _siftdown(heap, node2index, startpos, pos):
    ### BEGIN SOLUTION
    newitem = heap[pos]
    # Follow the path to the root, moving parents down until finding a place
    # newitem fits.
    while pos > startpos:
        parentpos = (pos - 1) >> 1 # Divide by 2 = floor( pos - 1 / 2)
        parent = heap[parentpos]
        if newitem < parent: # parent must be smaller
            heap[pos] = parent
            node2index[parent] = pos # track the position of the node
            pos = parentpos
            continue
        break # found a parent that is smaller
    heap[pos] = newitem
    node2index[newitem] = pos # track the position of node
    ### END SOLUTION

# ======== Modified code from heapq.py ====================================
# The child indices of heap index pos are already heaps, and we want to make
# a heap at index pos too.  We do this by bubbling the smaller child of
# pos up (and so on with that child's children, etc) until hitting a leaf,
# then using _siftdown to move the oddball originally at index pos into place.
def _siftup(heap, node2index, pos):
    ### BEGIN SOLUTION
    endpos = len(heap)
    startpos = pos
    newitem = heap[pos]
    # Bubble up the smaller child until hitting a leaf.
    childpos = 2*pos + 1    # leftmost child position
    while childpos < endpos:
        # Set childpos to index of smaller child.
        rightpos = childpos + 1
        if rightpos < endpos and not heap[childpos] < heap[rightpos]:
            childpos = rightpos
        # Move the smaller child up.
        heap[pos] = heap[childpos]
        node2index[heap[childpos]] = pos
        pos = childpos
        childpos = 2*pos + 1
    # The leaf at pos is empty now.  Put newitem there, and bubble it up
    # to its final resting place (by sifting its parents down).
    heap[pos] = newitem
    node2index[newitem] = pos
    _siftdown(heap, node2index, startpos, pos)
    ### END SOLUTION

# ======== Modified code from heapq.py ====================================
def heappush(heap, node2index, item):
    """
    Adds a new item to the heap and updates node2index dictionary
    to track the location of a hashable item in the heap

    Returns the updated node2index dictionary
    Push item onto heap, maintaining the heap invariant."""
    ### BEGIN SOLUTION
    heap.append(item)
    _siftdown(heap, node2index, 0, len(heap)-1)
    ### END SOLUTION
    return node2index

# ======== Modified code from heapq.py ====================================
def heappop(heap, node2index, retindex=0):
    """
    Removes the smallest item from the heap and returns it

    Pop the smallest item off the heap, maintaining the heap invariant."""
    lastelt = heap.pop()   # raises appropriate IndexError if heap is empty
    ### BEGIN SOLUTION
    if heap:
        returnitem = heap[retindex]
        heap[retindex] = lastelt
        node2index[lastelt] = retindex
        _siftup(heap, node2index, retindex)
        del node2index[returnitem]
        return returnitem
    del node2index[lastelt]
    ### END SOLUTION
    return lastelt


# ======== Modified code from heapq.py ====================================
def heapreplace(heap, node2index, olditem, newitem):
    """
    Finds and removes the given item from the heap. Updates the node2index dictionary due to the removal.

    Pop and return the current smallest value, and add the new item.

    This is more efficient than heappop() followed by heappush(), and can be
    more appropriate when using a fixed-size heap.  Note that the value
    returned may be larger than item!  That constrains reasonable uses of
    this routine unless written as part of a conditional replacement:

        if item > heap[0]:
            item = heapreplace(heap, item)
    """
    # TODO: Implement this function by reading Chapter 7 of Carmen's book and borrowing code from heapq.py
    ### BEGIN SOLUTION
    index = node2index[olditem]
    returnitem = heap[index]    # raises appropriate IndexError if heap is empty
    heap[index] = newitem
    del node2index[olditem]
    node2index[newitem] = index
    _siftup(heap, node2index, index)
    return returnitem
    ### END SOLUTION


class PriorityQueueUpdatable():
    '''Variant of Queue that retrieves open entries in priority order (lowest first).

    Entries are typically tuples of the form:  (priority number, data).
    '''

    def __init__(self):
        self.queue = []
        self.node2index = {}

    def empty(self):
        return len(self.queue) == 0

    def __len__(self):
        return len(self.queue)
    
    def __contains__(self, node):
        return node in self.node2index

    def put(self, item):
        self.node2index = heappush(self.queue, self.node2index, item)

    def get(self):
        node = heappop(self.queue, self.node2index)
        return node

    def replace(self, old_item, new_item):
        heapreplace(self.queue, self.node2index, old_item, new_item)

