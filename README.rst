Map generator
.............

:Stable release: unreleased

:Status: Experimental

:Maintainer: https://github.com/rlsosborne

:Description: Generate code sequences / lookup tables mapping keys to values.

.. image:: https://travis-ci.org/xcore/tool_map_generator.png?branch=master   :target: https://travis-ci.org/xcore/tool_map_generator

Key Features
============

* The map_generator executable takes a list of key, value pairs and it emits C
  code implementing the specified mapping.
* The map_generator library provides an API for generating mapping code
  sequences / lookup tables.

Description
===========

The map generator emits C code that maps from a set of integer keys to a set of
integer values. The crc instruction is used to map the set of keys on to a
dense set of integers that are then used as the index of a lookup table. A
brute force search is used to select the polynomial which minimizes the lookup
table size.

Support
=======

Issues may be submitted via the 'Issues' tab of the github repository
