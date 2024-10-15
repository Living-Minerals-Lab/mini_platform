<template>
    <div class="main">
        <h1>Basic Control</h1>
        <button @click="runRobot">Start</button>
        
        <br>
        
        <button>Stop</button>
        
        <br>
        Current Status
    </div>
</template>

<!-- <script lang="ts">
    export default{
        name: "MainPage"
    }
</script> -->

<script setup lang="ts" name="MainPage">
    import ROSLIB  from "roslib"

    const ros = new ROSLIB.Ros( {

    })

    // If there is an error on the backend, an 'error' emit will be emitted.
    ros.on('error', function(error: any) {
        console.log(error);
    });

    // Find out exactly when we made a connection.
    ros.on('connection', function() {
        console.log('Connection made!');

    });

    ros.on('close', function() {
        console.log('Connection closed.');

    });

    // Create a connection to the rosbridge WebSocket server.
    ros.connect('ws://localhost:9090');

    
    function runRobot () {
        console.log('hello')
    }

</script>

<style>
    .main {
        background-color: #3c68df;
        box-shadow: 0 0 10px;
        border-radius: 10px;
        padding: 20px;
    }

</style>