package com.esp32.coolp.mapper;

import com.esp32.coolp.dto.ReadingsDTO;
import com.esp32.coolp.entity.Readings;
import com.fasterxml.jackson.databind.ObjectMapper;
import org.mapstruct.*;

import java.util.List;

@Mapper(componentModel = "spring")
public interface ReadingsMapper {
    @Mapping(source = "id", target = "id")
    @Mapping(source = "voltage1", target = "voltage1")
    @Mapping(source = "current1", target = "current1")
    @Mapping(source = "power1", target = "power1")
    @Mapping(source = "frequency1", target = "frequency1")
    @Mapping(source = "voltage2", target = "voltage2")
    @Mapping(source = "current2", target = "current2")
    @Mapping(source = "power2", target = "power2")
    @Mapping(source = "frequency2", target = "frequency2")
    @Mapping(source = "voltage3", target = "voltage3")
    @Mapping(source = "current3", target = "current3")
    @Mapping(source = "power3", target = "power3")
    @Mapping(source = "frequency3", target = "frequency3")
    @Mapping(source = "sequence", target = "sequence")
    @Mapping(source = "createdAt", target = "createdAt")
    @Mapping(source = "deviceId", target = "deviceId")
    ReadingsDTO map(Readings readings);

    List<ReadingsDTO> map(List<Readings> readingsList);
}
