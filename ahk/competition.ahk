on := true
sleepTime := 50

$NumLock:: {
    global on
    on := !on
}

#HotIf on
Up:: Send "moveby(UP),`n"
Left:: Send "moveby(LEFT),`n"
Down:: Send "moveby(DOWN),`n"
Right:: Send "moveby(RIGHT),`n"

+Up:: Send "moveby(UP) & REVERSE,`n"
+Left:: Send "moveby(LEFT) & REVERSE,`n"
+Down:: Send "moveby(DOWN) & REVERSE,`n"
+Right:: Send "moveby(RIGHT) & REVERSE,`n"

^Up:: Send "moveby(0.7f * UP),`n"
^Left:: Send "moveby(0.7f * LEFT),`n"
^Down:: Send "moveby(0.7f * DOWN),`n"
^Right:: Send "moveby(0.7f * RIGHT),`n"

^+Up:: Send "moveby(0.7f * UP) & REVERSE,`n"
^+Left:: Send "moveby(0.7f * LEFT) & REVERSE,`n"
^+Down:: Send "moveby(0.7f * DOWN) & REVERSE,`n"
^+Right:: Send "moveby(0.7f * RIGHT) & REVERSE,`n"

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
