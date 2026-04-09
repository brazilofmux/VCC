#pragma once
/*
Copyright 2025 by the VCC Project Contributors.
This file is part of VCC (Virtual Color Computer).

    VCC (Virtual Color Computer) is free software: you can redistribute it
    and/or modify it under the terms of the GNU General Public License as
    published by the Free Software Foundation, either version 3 of the
    License, or (at your option) any later version.

    VCC (Virtual Color Computer) is distributed in the hope that it will be
    useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with VCC (Virtual Color Computer). If not, see
    <http://www.gnu.org/licenses/>.
*/

// EventHeap: A small min-heap of timed events for the emulation engine.
//
// Events are scheduled in nanoseconds relative to a master nanosecond
// counter that advances per scanline. When an event's deadline is reached,
// its handler fires and the event is rescheduled by adding its rearm delta.
//
// The heap is tiny (< 16 entries) so we use a simple sorted array rather
// than a proper heap structure. Insert is O(n) but n is ~8, so this is
// faster than the pointer-chasing overhead of a real heap.

#include <cstdint>
#include <functional>

struct ScheduledEvent
{
    double deadline;                    // nanos at which this event fires
    double rearm_delta;                 // nanos to add for rescheduling (0 = one-shot)
    void (*handler)();                  // callback when event fires
    const char* name;                   // for debugging
    bool enabled;                       // if false, event is skipped and not rescheduled
};

class EventHeap
{
public:
    static constexpr int MAX_EVENTS = 16;

    EventHeap() : count_(0) {}

    // Schedule a new recurring event. Returns an ID for later reference.
    int Schedule(const char* name, double deadline, double rearm_delta,
                 void (*handler)(), bool enabled = true)
    {
        if (count_ >= MAX_EVENTS) return -1;

        int id = count_++;
        events_[id].deadline = deadline;
        events_[id].rearm_delta = rearm_delta;
        events_[id].handler = handler;
        events_[id].name = name;
        events_[id].enabled = enabled;

        SortEvents();
        return FindById(id);
    }

    // Returns the deadline of the nearest enabled event, or a large value
    // if no events are scheduled.
    double NextDeadline() const
    {
        for (int i = 0; i < count_; i++)
        {
            if (events_[sorted_[i]].enabled)
                return events_[sorted_[i]].deadline;
        }
        return 1e18;
    }

    // Fire all events whose deadline <= now. Reschedule recurring events.
    // Returns the number of events fired.
    int FireExpired(double now)
    {
        int fired = 0;
        bool needs_sort = false;

        for (int i = 0; i < count_; i++)
        {
            int idx = sorted_[i];
            if (!events_[idx].enabled)
                continue;
            if (events_[idx].deadline > now)
                break;  // sorted, so no more expired events

            events_[idx].handler();
            fired++;

            if (events_[idx].rearm_delta > 0)
            {
                events_[idx].deadline += events_[idx].rearm_delta;
                needs_sort = true;
            }
            else
            {
                events_[idx].enabled = false;
            }
        }

        if (needs_sort)
            SortEvents();

        return fired;
    }

    // Subtract elapsed nanos from all deadlines. Call at end of each
    // scanline to keep deadline values from growing without bound.
    void AdvanceTime(double elapsed)
    {
        for (int i = 0; i < count_; i++)
        {
            events_[i].deadline -= elapsed;
        }
    }

    // Enable/disable an event by ID.
    void SetEnabled(int id, bool enabled)
    {
        if (id >= 0 && id < count_)
        {
            events_[id].enabled = enabled;
            SortEvents();
        }
    }

    // Update the rearm delta for an event.
    void SetRearmDelta(int id, double delta)
    {
        if (id >= 0 && id < count_)
            events_[id].rearm_delta = delta;
    }

    // Update the deadline for an event (e.g., when timer register changes).
    void SetDeadline(int id, double deadline)
    {
        if (id >= 0 && id < count_)
        {
            events_[id].deadline = deadline;
            SortEvents();
        }
    }

    // Reset all events.
    void Clear()
    {
        count_ = 0;
    }

    // Access event by ID (for inspection).
    const ScheduledEvent& GetEvent(int id) const { return events_[id]; }
    int Count() const { return count_; }

private:
    ScheduledEvent events_[MAX_EVENTS];
    int sorted_[MAX_EVENTS];    // indices into events_, sorted by deadline
    int count_;

    // Simple insertion sort. N is tiny.
    void SortEvents()
    {
        for (int i = 0; i < count_; i++)
            sorted_[i] = i;

        for (int i = 1; i < count_; i++)
        {
            int key = sorted_[i];
            double key_deadline = events_[key].enabled ? events_[key].deadline : 1e18;
            int j = i - 1;
            while (j >= 0)
            {
                double jd = events_[sorted_[j]].enabled ? events_[sorted_[j]].deadline : 1e18;
                if (jd <= key_deadline) break;
                sorted_[j + 1] = sorted_[j];
                j--;
            }
            sorted_[j + 1] = key;
        }
    }

    // Find the sorted position of a given event ID (for returning stable IDs).
    int FindById(int id) const
    {
        // IDs are indices into events_[], not sorted_[].
        // Just return the raw id since that's what we use externally.
        return id;
    }
};
