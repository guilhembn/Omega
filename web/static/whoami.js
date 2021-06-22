function check(){
    $.post("check.php", {}, (resp) =>{
        console.log(resp);
        location.reload();
        }

    );
}