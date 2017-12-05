pipeline {
  agent none
  stages {
    stage('Download Repo') {
      steps {
        git(url: 'https://github.com/wavelab/atl', branch: 'ros')
      }
    }
  }
}