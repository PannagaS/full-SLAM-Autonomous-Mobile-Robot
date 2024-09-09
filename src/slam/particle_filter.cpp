#include <utils/grid_utils.hpp>
//#include <utils/geometric.hpp> //check this include statement
#include <slam/particle_filter.hpp>
#include <slam/action_model.hpp>
#include <slam/occupancy_grid.hpp>
#include <mbot_lcm_msgs/pose2D_t.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <cassert>
#include <vector>
#include <chrono>
#include <utils/geometric/angle_functions.hpp>

ParticleFilter::ParticleFilter(int numParticles)
: kNumParticles_ (numParticles),
  samplingAugmentation_(0.5, 0.9, numParticles),
  distribution_quality(1),
  quality_reinvigoration_percentage(0.1)
{
    assert(kNumParticles_ > 1);
    posterior_.resize(kNumParticles_);
}


void ParticleFilter::initializeFilterAtPose(const mbot_lcm_msgs::pose2D_t& pose)
{
    ///////////// TODO: Implement your method for initializing the particles in the particle filter /////////////////
    double sampleWeight = 1.0/kNumParticles_;
    posteriorPose_ = pose;

    double std_xy = 0.02;
    double std_theta = 0.02;
    std::normal_distribution<> dist_xy(0, std_xy);
    std::normal_distribution<> dist_theta(0, std_theta);    

    for(auto& p: posterior_)
    {
        p.pose.x = posteriorPose_.x + dist_xy(numberGenerator_);
        p.pose.y = posteriorPose_.y + dist_xy(numberGenerator_);
        p.pose.theta = wrap_to_pi(posteriorPose_.theta + dist_theta(numberGenerator_));
        p.pose.utime = pose.utime;
        p.parent_pose = p.pose;
        p.weight = sampleWeight;
    }

}
//this is not needed imo
void ParticleFilter::initializeFilterRandomly(const OccupancyGrid& map)
{
    ///////////// TODO: Implement your method for initializing the particles in the particle filter /////////////////
    

}

void ParticleFilter::resetOdometry(const mbot_lcm_msgs::pose2D_t& odometry)
{
    actionModel_.resetPrevious(odometry);
}


mbot_lcm_msgs::pose2D_t ParticleFilter::updateFilter(const mbot_lcm_msgs::pose2D_t& odometry,
                                                        const mbot_lcm_msgs::lidar_t& laser,
                                                        const OccupancyGrid& map)
{
    bool hasRobotMoved = actionModel_.updateAction(odometry); //check if the bot actually moved
    // timer for this function
    if(hasRobotMoved)
    {
        auto start = std::chrono::high_resolution_clock::now();

        ParticleList prior = resamplePosteriorDistribution(map); // resample the distribution to find new weights
        ParticleList proposal = computeProposalDistribution(prior); // apply the action model by computing proposal distribution
        posterior_ = computeNormalizedPosterior(proposal, laser, map); //apply sensor model by computing normalized posterior
        posteriorPose_ = estimatePosteriorPose(posterior_);

        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end-start);
        std::cout << "Particle Filter time: " << duration.count() << " ms" << std::endl;
    }
    // std::cout << posterior_.size() << std::endl;
    posteriorPose_.utime = odometry.utime;
    return posteriorPose_;
}

mbot_lcm_msgs::pose2D_t ParticleFilter::updateFilterActionOnly(const mbot_lcm_msgs::pose2D_t& odometry)
{
    // Only update the particles if motion was detected. If the robot didn't move, then
    // obviously don't do anything.
    bool hasRobotMoved = actionModel_.updateAction(odometry);

    if(hasRobotMoved)
    {
        //auto prior = resamplePosteriorDistribution(); //this was not there in the video
        ParticleList proposal = computeProposalDistribution(posterior_); //runs applyAction on the sample
        posterior_ = proposal;
    }
    posteriorPose_ = odometry;

    return posteriorPose_;
}


mbot_lcm_msgs::pose2D_t ParticleFilter::poseEstimate(void) const
{
    return posteriorPose_;
}


mbot_lcm_msgs::particles_t ParticleFilter::particles(void) const
{
    mbot_lcm_msgs::particles_t particles;
    particles.num_particles = posterior_.size();
    particles.particles = posterior_;
    return particles;
}


// ParticleList ParticleFilter::resamplePosteriorDistribution(const OccupancyGrid& map,
//                                                            const bool keep_best,
//                                                            const bool reinvigorate)
// {
//     //////////// TODO: Implement your algorithm for resampling from the posterior distribution ///////////////////
//     //used in sensor model

//     ParticleList prior = posterior_;

//     double sampleWeight = 1.0/kNumParticles_;
//     std::random_device rd;
//     std::mt19937 generator(rd());
//     std::normal_distribution<> dist(0.0, 0.04); //0.04 => std deviation of 4cm 

//     for(auto& p:prior)
//     {
//         p.pose.x = posteriorPose_.x + dist(generator);
//         p.pose.y = posteriorPose_.y + dist(generator);
//         p.pose.theta = posteriorPose_.theta + dist(generator);
//         p.pose.utime = posteriorPose_.utime;
//         p.parent_pose = posteriorPose_;
//         p.weight = sampleWeight;
//     }
//     return prior;
// }

ParticleList ParticleFilter::resamplePosteriorDistribution(const OccupancyGrid& map,
                                                           const bool keep_best,
                                                           const bool reinvigorate)
{
    //////////// TODO: Implement your algorithm for resampling from the posterior distribution ///////////////////

    ParticleList prior;

    double sampleWeight = 1.0/(float)kNumParticles_;
    std::random_device rd;
    std::mt19937 generator(rd());
    std::uniform_real_distribution<> dist_sample(0.0, sampleWeight);
    double r = dist_sample(generator);
    double c = posterior_[0].weight;
    int i = 1;
    for (int m=1; m<=kNumParticles_; ++m)
    {
        double U = r + (m-1) * sampleWeight;
        while (U > c)
        {
            i = i + 1;
            c = c + posterior_[i-1].weight;
        }
        prior.push_back(posterior_[i-1]);
    }
    return prior;
}


void ParticleFilter::reinvigoratePriorDistribution(ParticleList& prior)
{
    // Augmentation: if sensor model suspects an average particle quality of
    //      less than 15%, invigorate
    if (distribution_quality < 0.15)  // TODO: make 0.15 a parameter
    {
        int count = 0;
        int max_count = floor(quality_reinvigoration_percentage * prior.size());

        std::random_device rd;
        std::default_random_engine generator(rd());
        auto ud01 = std::uniform_real_distribution<double>(0.0, 1.0);
        int step = std::max<int>(1, floor(ud01(generator) * prior.size() / max_count));

        for (int i = 0; i < max_count; i++)
        {
            prior[i*step] = randomPoseGen_.get_particle();
        }

    }
}


ParticleList ParticleFilter::computeProposalDistribution(const ParticleList& prior)
{
    //////////// TODO: Implement your algorithm for creating the proposal distribution by sampling from the ActionModel
    
    ParticleList proposal; //std::vector<particle_t>proposal;
    
    for(auto & p: prior)
    {
        proposal.push_back(actionModel_.applyAction(p)); //check datatype

    }
    return proposal; // -> in video
}


ParticleList ParticleFilter::computeNormalizedPosterior(const ParticleList& proposal,const mbot_lcm_msgs::lidar_t& laser,const OccupancyGrid& map)
{
    /////////// TODO: Implement your algorithm for computing the normalized posterior distribution using the
    ///////////       particles in the proposal distribution

    std::vector<mbot_lcm_msgs::particle_t> posterior;
    double sumWeights=0;
    for(auto& p:proposal){
        auto weighted = p;
        weighted.weight = sensorModel_.likelihood(weighted, laser, map);
        // if (weighted.weight == 0) {
        //     std::cout << "weights" << weighted.weight << std::endl;
        // }
        sumWeights += weighted.weight;
        posterior.push_back(weighted);
    }

    for(auto& p:posterior)
    {
        p.weight/=sumWeights;
    }

    return posterior;
    //return ParticleList();  // Placeholder
}


mbot_lcm_msgs::pose2D_t ParticleFilter::estimatePosteriorPose(const ParticleList& posterior)
{
    //////// TODO: Implement your method for computing the final pose estimate based on the posterior distribution
    // Figure out which pose to take for the posterior pose
    // Weighted average is simple, but could be very bad
    // Maybe only take the best x% and then average.


    // ParticleList posteriro_sorted = posterior;
    // std::sort(posteriro_sorted.begin(),
    //           posteriro_sorted.end(),
    //           [](const mbot_lcm_msgs::particle_t a, const mbot_lcm_msgs::particle_t b) {
    //                             return a.weight > b.weight; // Sort in descending order
    //                             }
    // );

    // // take only 25% of the top candidates among all particles
    // int num_25_particles = posteriro_sorted.size() * .10;
    // ParticleList topParticles(posteriro_sorted.begin(), posteriro_sorted.begin() + num_25_particles);

    // mbot_lcm_msgs::pose2D_t pose = computeParticlesAverage(topParticles);
    mbot_lcm_msgs::pose2D_t pose = computeParticlesAverage(posterior);
    return pose;
}

mbot_lcm_msgs::pose2D_t ParticleFilter::computeParticlesAverage(const ParticleList& particles_to_average)
{
    mbot_lcm_msgs::pose2D_t avg_pose;
    avg_pose.x = 0.0;
    avg_pose.y = 0.0;
    avg_pose.theta = 0.0;
    double sum_weight = 0.0;

    // Aux variables to compute theta average
    double theta_x = 0.0;
    double theta_y = 0.0;
    for (auto &&p : particles_to_average)
    {
        avg_pose.x += p.weight * p.pose.x;
        avg_pose.y += p.weight * p.pose.y;
        theta_x += p.weight * std::cos(p.pose.theta);
        theta_y += p.weight * std::sin(p.pose.theta);

        sum_weight += p.weight;
    }
    avg_pose.theta = std::atan2(theta_y, theta_x);

    return avg_pose;
}
