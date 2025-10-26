on := true

#HotIf on
Up:: Send "moveby(UP),`n"
Left:: Send "moveby(LEFT),`n"
Down:: Send "moveby(DOWN),`n"
Right:: Send "moveby(RIGHT),`n"

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
