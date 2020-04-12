<script type="text/javascript">
    var tet= document.getElementById("text");
    var butt= document.getElementById("btn");
    var tet_li= document.getElementById("text_li");
 
    butt.onclick=function msg(){
       var Li= document.createElement("li");
        Li.innerHTML= tet.value;
        tet_li.appendChild(Li);
  }
       </script>