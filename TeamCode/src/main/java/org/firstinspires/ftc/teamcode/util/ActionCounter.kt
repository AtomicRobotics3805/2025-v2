package org.firstinspires.ftc.teamcode.util

import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.SequentialAction
import page.j5155.expressway.ftc.ActionRunner

class ActionCounter(val actionRunner: ActionRunner) {

    private var count = 0    
    
    /**
     * Attempts to find an approximation for the number of actions currently running, including those
     * within a ParallelAction or a SequentialAction. Note that this is only an approximation, because 
     * they do not expose their `actions` field, so we cannot see which actions are STILL running.
     * This approximation instead finds the number of actions that COULD STILL BE running. 
     */
    fun findActiveActions(): Int {
        count = 0
        
        actionRunner.runningActions.forEach { 
            count(it)
        }
        
        return count
    }

    /**
     * Attempt to recursively count the number of actions that are currently running.
     */
    private fun count(action: Action) {
        if (action::class != ParallelAction::class && action::class != SequentialAction::class) {
            count++
        } else {
            if (action::class == ParallelAction::class) {
                count++
                (action as ParallelAction).initialActions.forEach {
                    count(it)
                }
            } else if (action::class == SequentialAction::class) {
                count++
                (action as SequentialAction).initialActions.forEach {
                    if (it::class == ParallelAction::class) {
                        count(it)
                    }
                }
            }
        }
    }
}