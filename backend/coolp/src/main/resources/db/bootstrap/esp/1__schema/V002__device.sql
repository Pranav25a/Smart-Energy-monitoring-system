--
-- Table structure for table `device`
--
DROP TABLE IF EXISTS `device`;
CREATE TABLE `device`
(
    `id`                            bigint(20) NOT NULL AUTO_INCREMENT,
    `name`                          VARCHAR(100) DEFAULT NULL,
    `created_at`                    TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    PRIMARY KEY (`id`),
) ENGINE=InnoDB DEFAULT CHARSET=utf8 COLLATE utf8_unicode_ci;
