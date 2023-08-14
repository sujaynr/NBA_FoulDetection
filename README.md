<!DOCTYPE html>
<html>
<head>
<style>
    .video-container {
        display: flex;
        justify-content: center;
        align-items: flex-start;
        gap: 20px;
    }
    .video-frame {
        width: 50%;
    }
</style>
</head>
<body>

<div class="video-container">
    <div class="video-frame">
        <video controls width="100%" height="auto">
            <source src="demo.mp4" type="video/mp4">
            Your browser does not support the video tag.
        </video>
    </div>
    <div class="video-frame">
        <video controls width="100%" height="auto">
            <source src="output.mp4" type="video/mp4">
            Your browser does not support the video tag.
        </video>
    </div>
</div>

</body>
</html>
