package com.esp32.coolp.entity;

import lombok.AllArgsConstructor;
import lombok.Data;
import lombok.NoArgsConstructor;

import jakarta.persistence.*;
import org.hibernate.annotations.Fetch;
import org.hibernate.annotations.FetchMode;

import java.sql.Timestamp;


@Data
@NoArgsConstructor
@AllArgsConstructor
@Entity
@Table(name = "readings")
public class Readings {

    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
     Long id;
    @Column(name = "voltage1")
     Float voltage1;
    @Column(name = "voltage2")
    Float voltage2;
    @Column(name = "voltage3")
    Float voltage3;
    @Column(name = "current1")
     Float current1;
    @Column(name = "current2")
    Float current2;
    @Column(name = "current3")
    Float current3;
    @Column(name = "power1")
     Float power1;
    @Column(name = "power2")
    Float power2;
    @Column(name = "power3")
    Float power3;
    @Column(name = "powerunit")
    Float powerunit;
    @Column(name = "frequency1")
     Float frequency1;
    @Column(name = "frequency2")
    Float frequency2;
    @Column(name = "frequency3")
    Float frequency3;
    @Column(name = "sequence")
    Boolean sequence;
    @Column(name = "created_at")
     Timestamp createdAt;
    @Column(name = "device_id")
     Long deviceId;

    @ManyToOne
    @Fetch(value = FetchMode.SELECT)
    @JoinColumn(name = "device_id", referencedColumnName = "id", insertable = false, updatable = false)
    private Device device;

}
