package com.esp32.coolp.service;

import com.esp32.coolp.entity.Device;
import com.esp32.coolp.repository.DeviceRepository;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Service;

import java.util.List;

@Service
public class DeviceService {

    @Autowired
    DeviceRepository deviceRepository;

//    public List<Device> findByName(String name){
////        return deviceRepository.findAllByNameContaining(name);
//    }

    public List<Device> findAll(){
        return deviceRepository.findAll();
    }
}
