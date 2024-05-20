package com.esp32.coolp.dto;

import jakarta.persistence.Column;
import jakarta.persistence.GeneratedValue;
import jakarta.persistence.GenerationType;
import lombok.AllArgsConstructor;
import lombok.Data;
import lombok.NoArgsConstructor;

import java.sql.Timestamp;

@Data
@NoArgsConstructor
@AllArgsConstructor
public class ReadingsDTO {

    Long id;

    Float voltage1;

    Float current1;

    Float power1;

    Float frequency1;
    Float voltage2;

    Float current2;

    Float power2;

    Float frequency2;
    Float voltage3;

    Float current3;

    Float power3;

    Float frequency3;

    Boolean sequence;


    Timestamp createdAt;

    Long deviceId;

}
