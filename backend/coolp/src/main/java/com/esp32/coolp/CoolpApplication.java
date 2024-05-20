package com.esp32.coolp;

import org.springframework.boot.SpringApplication;
import org.springframework.boot.autoconfigure.SpringBootApplication;
import org.springframework.boot.autoconfigure.domain.EntityScan;

@SpringBootApplication
@EntityScan("com.esp32.coolp.entity")
public class CoolpApplication {

    public static void main(String[] args) {
        SpringApplication.run(CoolpApplication.class, args);
    }

}
