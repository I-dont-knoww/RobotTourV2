on := true
sleepTime := 50

offsetAmount := "5.0f"
previousInput := ""

GetOpposite(direction) {
    if (direction == "UP")
        return "DOWN"
    if (direction == "LEFT")
        return "RIGHT"
    if (direction == "DOWN")
        return "UP"
    if (direction == "RIGHT")
        return "LEFT"
    return "???"
}

DiagonalImplementation(input, output, reverse, small) {
    Send "// " input " => " output
    if (reverse)
        Send " (reversed)"
    Send "`n"

    Sleep(sleepTime)

    Send "moveby(" input ") & OFFSET_ONCE(" offsetAmount " * " output " {+} 25.0f * " GetOpposite(input) ")"
    if (reverse)
        Send " & REVERSE"
    Send ",`n"

    Sleep(sleepTime)

    Send "moveby({{}{}}) & OFFSET_ONCE(25.0f * " output " {+} " offsetAmount " * " GetOpposite(input) ")"
    if (reverse)
        Send " & REVERSE"
    Send ",`n"

    Sleep(sleepTime)

    if (small)
        Send "moveby(0.7f * " output ")"
    else
        Send "moveby(" output ")"

    if (reverse)
        Send " & REVERSE"
    Send ",`n"
}

Diagonal(direction, reverse, small) {
    global previousInput

    if (previousInput == "")
        previousInput := direction
    else {
        DiagonalImplementation(previousInput, direction, reverse, small)
        previousInput := ""
    }
}

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

!Up:: Diagonal("UP", false, false)
!Left:: Diagonal("LEFT", false, false)
!Down:: Diagonal("DOWN", false, false)
!Right:: Diagonal("RIGHT", false, false)

!+Up:: Diagonal("UP", true, false)
!+Left:: Diagonal("LEFT", true, false)
!+Down:: Diagonal("DOWN", true, false)
!+Right:: Diagonal("RIGHT", true, false)

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
