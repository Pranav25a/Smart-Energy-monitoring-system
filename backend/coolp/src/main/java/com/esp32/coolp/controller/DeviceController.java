package com.esp32.coolp.controller;

import com.esp32.coolp.entity.Device;
import com.esp32.coolp.service.DeviceService;
import lombok.AllArgsConstructor;
import lombok.Data;
import lombok.NoArgsConstructor;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;

import java.util.List;

@Data
@NoArgsConstructor
@AllArgsConstructor
@RestController
@RequestMapping(path = "/device")
public class DeviceController {

    @Autowired
    DeviceService deviceService;

    @GetMapping()
    public List<Device> index(){
        return deviceService.findAll();
    }

}
