on := true

#HotIf on
Up:: Send "moveby(UP),`n"
Left:: Send "moveby(LEFT),`n"
Down:: Send "moveby(DOWN),`n"
Right:: Send "moveby(RIGHT),`n"

+Up:: Send "moveby(UP) & REVERSE,`n"
+Left:: Send "moveby(LEFT) & REVERSE,`n"
+Down:: Send "moveby(DOWN) & REVERSE,`n"
+Right:: Send "moveby(RIGHT) & REVERSE,`n"

^Up:: Send "moveby(0.5f * UP),`n"
^Left:: Send "moveby(0.5f * LEFT),`n"
^Down:: Send "moveby(0.5f * DOWN),`n"
^Right:: Send "moveby(0.5f * RIGHT),`n"

^+Up:: Send "moveby(0.5f * UP) & REVERSE,`n"
^+Left:: Send "moveby(0.5f * LEFT) & REVERSE,`n"
^+Down:: Send "moveby(0.5f * DOWN) & REVERSE,`n"
^+Right:: Send "moveby(0.5f * RIGHT) & REVERSE,`n"

$Backspace:: {
    Send "+{Up}"
    Sleep(50)
    Send "{Backspace}"
}
#HotIf

$NumLock:: {
    global on
    on := !on
}

[:: ExitApp
