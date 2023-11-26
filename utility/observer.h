#pragma once

// EPOS Observer Utility Declarations

// Observation about the lack of virtual destructors in the following classes:
// Observed x Observer is used in mediators, so they appear very early in the system.
// To be more precise, they are used in SETUP, where we cannot yet handle a heap.
// Since the purpose of the destructors is only to trace the classes, we accepted to
// declare them as non-virtual. But it must be clear that this is one of the few uses
// for them.


// Observer x Observed
class Observer;

class Observed;