package com.esp32.coolp.dto;

import java.sql.Timestamp;

import lombok.AllArgsConstructor;
import lombok.Data;
import lombok.NoArgsConstructor;

@Data
@NoArgsConstructor
@AllArgsConstructor
public class PowerDTO {

    Float powerunit;

    Timestamp createdAt;
}
