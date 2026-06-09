# SPDX-FileCopyrightText: 2022 Uwe Fechner
# SPDX-License-Identifier: MIT

using REPL.TerminalMenus: RadioMenu, request

options = [
    "ram_air_kite = include(\"ram_air_kite.jl\")",
    "auto_parking_ram_air = include(\"auto_parking_ram_air.jl\")",
    "quit",
]

function example_menu(options)
    active = true
    while active
        menu = RadioMenu(options; pagesize=8)
        choice = request("\nChoose example to run or `q` to quit: ", menu)

        if choice != -1 && choice != length(options)
            expr = Meta.parse(options[choice])
            eval(expr)
        else
            println("Left menu. Press <ctrl><d> to quit Julia!")
            active = false
        end
    end
end

example_menu(options)
