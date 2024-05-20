package com.esp32.coolp.controller;

import lombok.RequiredArgsConstructor;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;

@RestController
@RequestMapping(path = "/home")
@RequiredArgsConstructor(onConstructor = @__(@Autowired))
public class HelloWorld {
    @GetMapping()
    public String Hello(){
        return "hello World";
    }
}
