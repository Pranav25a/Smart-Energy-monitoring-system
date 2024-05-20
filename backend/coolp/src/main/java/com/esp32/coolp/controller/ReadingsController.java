package com.esp32.coolp.controller;

import com.esp32.coolp.dto.PowerDTO;
import com.esp32.coolp.dto.ReadingsDTO;
import com.esp32.coolp.entity.Readings;
import com.esp32.coolp.mapper.ReadingsMapper;
import com.esp32.coolp.service.ReadingsService;
import org.springframework.format.annotation.DateTimeFormat;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;

import java.sql.Timestamp;
import java.time.Instant;
import java.time.LocalDateTime;
import java.time.ZoneId;
import java.time.ZonedDateTime;
import java.util.List;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.RequestParam;


@RestController
@RequestMapping(path = "/readings")
@CrossOrigin(origins = "*", allowedHeaders = "*")
public class ReadingsController {

    private final ReadingsService readingsService;
    private final ReadingsMapper readingsMapper;

    public ReadingsController(ReadingsService readingsService, ReadingsMapper readingsMapper) {
        this.readingsService = readingsService;
        this.readingsMapper = readingsMapper;
    }

    @GetMapping(path = "/{id}")
    public List<ReadingsDTO> getByDeviceId(@PathVariable(value = "id") Long id) {
        List<Readings> readings = readingsService.findAllByDeviceId(id);
        return readingsMapper.map(readings);
    }
    @GetMapping(path = "/{id}/currentData")
    public ReadingsDTO getCurrentData(@PathVariable(value = "id") Long id) {
        Readings readings = readingsService.findCurrentData(id);
        return readingsMapper.map(readings);
    }

    @GetMapping(path = "/{deviceId}/range")
    public List<ReadingsDTO> getByDeviceIdAndDateRange(
            @PathVariable(value = "deviceId") Long deviceId,
            @RequestParam(name = "startDate") @DateTimeFormat(iso = DateTimeFormat.ISO.DATE_TIME) LocalDateTime startDate,
            @RequestParam(name = "endDate") @DateTimeFormat(iso = DateTimeFormat.ISO.DATE_TIME) LocalDateTime endDate) {

        Timestamp startTimestamp = Timestamp.valueOf(startDate);
        Timestamp endTimestamp = Timestamp.valueOf(endDate);

        List<Readings> readings = readingsService.findAllByDeviceIdAndDateRange(deviceId, startTimestamp, endTimestamp);
        System.out.println(readings.get(0));
        System.out.println(readingsMapper.map(readings.get(0)));
        System.out.println(readingsMapper.map(readings));
        return readingsMapper.map(readings);
    }

    // @GetMapping(path = "/powerunit")
    // public PowerDTO powerunit(){
    //     Readings readings = readingsService.findCurrentData(1);

    // }
    public String getMethodName(@RequestParam String param) {
        return new String();
    }
    
    @GetMapping(path = "/checking/{content}")
    public String chekcer(
            @PathVariable(value = "content") String content
    ){
    System.out.println(content);
    return content;
    }
    @PostMapping
    public ResponseEntity<Readings> createReading(@RequestBody Readings readings) {
        if (readings.getCreatedAt() == null) {
            Instant nowUtc = Instant.now();
            ZonedDateTime nowIst = nowUtc.atZone(ZoneId.of("UTC")).withZoneSameInstant(ZoneId.of("Asia/Kolkata"));
            Timestamp timestampIst = Timestamp.from(nowIst.toInstant());
            readings.setCreatedAt(timestampIst);
        }
        Readings createdReadings = readingsService.save(readings);
        return new ResponseEntity<>(createdReadings, HttpStatus.CREATED);
    }

}
