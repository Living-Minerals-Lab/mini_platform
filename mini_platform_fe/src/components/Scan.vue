<template>
    <div class="scan">
        <h1>Sample Scan</h1>
        Sample Length: <input></input>
        <br>
        Sample Width: <input></input>
        
        <br>
        #Points in Length: <input type="integer" v-bind:value="3"></input>
        <br>

        #Points in Width: <input type="integer" v-bind:value="4"></input>
        
        <br>
        Grid Preview
    </div>
</template>

<script lang="ts">
    export default{
        name: "Scan"
    }
</script>

<style>
    .scan {
        background-color: #3cdfd7;
        box-shadow: 0 0 10px;
        border-radius: 10px;
        padding: 20px;
    }

</style>