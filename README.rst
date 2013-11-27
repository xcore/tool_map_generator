Map generator
.............

:Stable release: unreleased

:Status: Experimental

:Maintainer: https://github.com/rlsosborne

:Description: Generate code sequences / lookup tables mapping keys to values.

Key Features
============

* The map_generator executable takes a file of key, value pairs and outputs C
  code that implements the specified mapping.
* The map_generator library provides an API for generating mapping code
  sequences / lookup tables.

Description
===========

The map_generator executable outputs C code that implements a mapping from a set
of integer keys to a set of integer values. The crc instruction is used to map
the set of keys on to a dense set of integers that are used as the index of a
lookup table. A brute force search is used to find the polynomial which
minimizes the lookup table size.

Support
=======

Issues may be submitted via the 'Issues' tab of the github repository
