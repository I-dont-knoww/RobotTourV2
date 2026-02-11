#SingleInstance Force
on := true
sleepTime := 50
global innovationOffsetted := false

$NumLock:: {
    global on
    on := !on
}

#HotIf on

; Up:: Send "moveby(UP),`n"
; Left:: Send "moveby(LEFT),`n"
; Down:: Send "moveby(DOWN),`n"
; Right:: Send "moveby(RIGHT),`n"

; +Up:: Send "moveby(UP) & REVERSE,`n"
; +Left:: Send "moveby(LEFT) & REVERSE,`n"
; +Down:: Send "moveby(DOWN) & REVERSE,`n"
; +Right:: Send "moveby(RIGHT) & REVERSE,`n"

; ^Up:: Send "moveby(0.7f * UP),`n"
; ^Left:: Send "moveby(0.7f * LEFT),`n"
; ^Down:: Send "moveby(0.7f * DOWN),`n"
; ^Right:: Send "moveby(0.7f * RIGHT),`n"

; ^+Up:: Send "moveby(0.7f * UP) & REVERSE,`n"
; ^+Left:: Send "moveby(0.7f * LEFT) & REVERSE,`n"
; ^+Down:: Send "moveby(0.7f * DOWN) & REVERSE,`n"
; ^+Right:: Send "moveby(0.7f * RIGHT) & REVERSE,`n"

; INNOVATION CHALLENGE
innovationFunction(dirString, realMoveString){
    global innovationOffsetted
    if (innovationOffsetted) {
        Send "moveby(0.0001f * " . dirString . "), `n"
    }
    innovationOffsetted := false
    Send realMoveString . ",`n"
} 

Up:: innovationFunction("UP", "moveby(UP)")
Down:: innovationFunction("DOWN", "moveby(DOWN)")
Right:: innovationFunction("RIGHT", "moveby(RIGHT)")
Left:: innovationFunction("LEFT", "moveby(LEFT)")

+Up:: innovationFunction("UP", "moveby(UP) & REVERSE")
+Left:: innovationFunction("LEFT", "moveby(LEFT) & REVERSE")
+Down:: innovationFunction("DOWN", "moveby(DOWN) & REVERSE")
+Right:: innovationFunction("RIGHT", "moveby(RIGHT) & REVERSE")

^Up:: innovationFunction("UP", "moveby(0.7f * UP)")
^Left:: innovationFunction("LEFT", "moveby(0.7f * LEFT)")
^Down:: innovationFunction("DOWN", "moveby(0.7f * DOWN)")
^Right:: innovationFunction("RIGHT", "moveby(0.7f * RIGHT)")

^+Up:: innovationFunction("UP", "moveby(0.7f * UP) & REVERSE")
^+Left:: innovationFunction("LEFT", "moveby(0.7f * LEFT) & REVERSE")
^+Down:: innovationFunction("DOWN", "moveby(0.7f * DOWN) & REVERSE")
^+Right:: innovationFunction("RIGHT", "moveby(0.7f * RIGHT) & REVERSE")

!Up::{
    global innovationOffsetted
    Send "moveby(UP) & OFFSET_ONCE(25.0f * DOWN),`n"
    innovationOffsetted := true
}
!Down::{
    global innovationOffsetted
    Send "moveby(DOWN) & OFFSET_ONCE(25.0f * UP),`n"
    innovationOffsetted := true
}
!Left::{
    global innovationOffsetted
    Send "moveby(LEFT) & OFFSET_ONCE(25.0f * RIGHT),`n"
    innovationOffsetted := true
}
!Right::{
    global innovationOffsetted
    Send "moveby(RIGHT) & OFFSET_ONCE(25.0f * LEFT),`n"
    innovationOffsetted := true
}

;----

`:: {
    Send("{Up}")
    Sleep(sleepTime)
    Send("^{Right}")
    Sleep(sleepTime)
    Send("+^{\}")
    Sleep(sleepTime)
    Send("+{Home}")
    Sleep(sleepTime)
    Send("+^{Right}")
    Sleep(sleepTime)
    Send("+{Right}")
    Sleep(sleepTime)
    Send("^{c}")
    Sleep(sleepTime)
    Send("+^{k}")
    Sleep(sleepTime)
    Send("{Up}")
    Sleep(sleepTime)
    Send("^{Right}")
    Sleep(sleepTime)
    Send("+^{\}")
    Sleep(sleepTime)
    Send(" {+} ")
    Sleep(sleepTime)
    Send("^{v}")
    Sleep(sleepTime)
    Send("{Down}")
    Sleep(sleepTime)
    Send("{Left}")
    Sleep(sleepTime)
    Send("{Right}")
}

$Backspace:: {
    Send("+{Up}")
    Sleep(sleepTime)
    Send("{Backspace}")
}
#HotIf

[:: ExitApp