In the realm of sports, basketball emerges as a fascinating area for the applications of AI, specifically, human body pose detection. While the technology behind human body pose detection reaches far beyond the court, with potential applications across various domains, I'm passionate about utilizing the state-of-the-art models to accurately call fouls and other illegal contact within the game.

Accurate foul calling is crucial for maintaining fairness, competitiveness, and the integrity of the sport. Integrating human body pose detection into basketball, though, is no simple task. While these models excel at capturing movements on a 2D scale, predicting 3D positions and handling collisions during fast-paced play present challenges.

To address this, I've taken a balanced approach using both 2D and 3D pose detection. By combining these dimensions, I've developed a method that uses the MMPOSE 2D inferencer to identify frames with close player interaction, indicating possible fouls. Then, the MMPOSE 3D model examines players' arm positions, shedding light on instances of contact.

Looking ahead, we envision a future where technology elevates the game while preserving the human touch.

<div style="display: flex; justify-content: center; align-items: center; gap: 20px;">
    <div>
        <img src="output.gif" alt="Video 2" width="800">
    </div>
</div>
