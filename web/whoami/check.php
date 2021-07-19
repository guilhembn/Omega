<?php
include "../private/addresses.php";
$level = (int)file_get_contents("../private/who_am_i_level", LOCK_EX);
if (isset($_SERVER['REMOTE_ADDR']) && in_array($_SERVER['REMOTE_ADDR'], $addresses[$level])){
    file_put_contents("../private/who_am_i_level", $level + 1, LOCK_EX);
}else{
    file_put_contents("../private/who_am_i_level", 0, LOCK_EX);
}